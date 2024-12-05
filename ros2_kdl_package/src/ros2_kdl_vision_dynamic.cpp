#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include <chrono>

using namespace KDL;
using namespace std::chrono_literals;

class CircularTrajectoryNode : public rclcpp::Node
{
public:
    CircularTrajectoryNode() : Node("circular_trajectory_node"), t_(0.0)
    {
        // Initialize robot model
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        auto parameter = parameters_client->get_parameters({"robot_description"});
        
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct kdl tree");
            return;
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);
        
        // Set joint limits
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -3.05;
        q_max.data << 2.96, 2.09, 2.96, 2.09, 2.96, 2.09, 3.05;
        robot_->setJntLimits(q_min, q_max);

        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_efforts_.resize(nj);

        // Define camera-end effector transformation
        KDL::Frame ee_T_cam;
        ee_T_cam.M = KDL::Rotation::RotY(M_PI/2) * KDL::Rotation::RotZ(-M_PI/2);
        ee_T_cam.p = KDL::Vector(0, 0, 0.025);
        robot_->addEE(ee_T_cam);

        // Create subscribers and publishers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CircularTrajectoryNode::joint_state_callback, this, std::placeholders::_1));

        marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10,
            std::bind(&CircularTrajectoryNode::marker_pose_callback, this, std::placeholders::_1));

        cmd_effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands", 10);

        // Control loop timer (20ms for effort control)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&CircularTrajectoryNode::control_loop, this));

        // Initialize controller
        controller_ = std::make_unique<KDLController>(*robot_);

        // Wait for first joint states
        while(!joint_states_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for joint states...");
            rclcpp::spin_some(this->get_node_base_interface());
        }

        // Initialize robot state
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        init_cart_pose_ = robot_->getEEFrame();

        // Trajectory parameters
        trajectory_duration_ = 5;  // Total duration
        acc_time_ = 1;           // Acceleration time for trapezoidal profile
        
        // Initialize circular trajectory
        Eigen::Vector3d start_pos(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));
        trajectory_radius_ = 0.1;
        
        // Create circular trajectory with trapezoidal velocity profile
        planner_ = std::make_unique<KDLPlanner>(
            trajectory_duration_,
            acc_time_,
            start_pos,
            trajectory_radius_
        );

        RCLCPP_INFO(this->get_logger(), "Circular trajectory node initialized");
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->position.size(); i++) {
            joint_positions_.data[i] = msg->position[i];
            joint_velocities_.data[i] = msg->velocity[i];
            joint_efforts_.data[i] = msg->effort[i];
        }
        joint_states_received_ = true;
    }

    void marker_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        marker_pose_ = *msg;
        marker_pose_received_ = true;
    }

void control_loop()
{
    if (!joint_states_received_ || !marker_pose_received_) return;

    robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
    KDL::Frame current_pose = robot_->getEEFrame();

    if (t_ <= trajectory_duration_) {
        // Get trajectory point for position
        trajectory_point p = planner_->compute_trajectory(t_);

        // Look-at-point computation
        Eigen::Vector3d c_P_o = toEigen(KDL::Vector(
            marker_pose_.pose.position.x,
            marker_pose_.pose.position.y,
            marker_pose_.pose.position.z
        ));
        
        // Compute s and desired s
        double c_P_o_norm = c_P_o.norm();
        Eigen::Vector3d s = c_P_o / c_P_o_norm;
        Eigen::Vector3d s_d(0, 0, 1);

        // Get camera Jacobian and current rotation
        KDL::Jacobian J_cam = robot_->getEEJacobian();
        Eigen::Matrix3d R_c = toEigen(robot_->getEEFrame().M);

        // Compute interaction matrix L
        Eigen::Matrix<double, 3, 6> L = Eigen::Matrix<double, 3, 6>::Zero();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d ssT = s * s.transpose();
        L.block<3,3>(0,0) = -1.0/c_P_o_norm * (I - ssT);
        L.block<3,3>(0,3) = skew(s);

        // Create desired pose and twist
        KDL::Frame desired_pose = current_pose;
        desired_pose.p = toKDL(p.pos);

        // Compute orientation error velocity command
        Eigen::Vector3d e_o = s_d - s;

        KDL::Twist desired_vel;
        desired_vel.vel = toKDL(p.vel);
        desired_vel.rot = toKDL(e_o); // Use orientation error for rotational velocity

        KDL::Twist desired_acc;
        desired_acc.vel = toKDL(p.acc);
        desired_acc.rot = KDL::Vector::Zero();

        // Compute torques with both position and orientation control
        Eigen::VectorXd tau = controller_->idCntr(
            desired_pose,
            desired_vel, 
            desired_acc,
            10.0,  // Kp position
            5.0,  // Ko orientation
            20.0,   // Kdp velocity
            10.0    // Kdo angular velocity
        );
        // Computing LJ and its pseudoinverse
        Eigen::MatrixXd LJ = L * J_cam.data;
        Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();

        // Compute null space projector
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(7,7) - LJ_pinv * LJ;
        Eigen::VectorXd v_orient = 1.0 * LJ_pinv * s_d;
        tau += robot_->getJsim() * N * v_orient;
        // Publish command
        auto effort_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        effort_msg->data = std::vector<double>(tau.data(), tau.data() + tau.size());
        cmd_effort_pub_->publish(std::move(effort_msg));

        t_ += 0.01;
    

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "t: %.2f, tau: [%.3f, %.3f, %.3f,%.3f, %.3f, %.3f, %.3f], e_o: [%.3f, %.3f, %.3f]",
            t_,
            tau(0), tau(1), tau(2), tau(3), tau(4), tau(5), tau(6),
            e_o[0], e_o[1], e_o[2]
        );
    }
    else if (!holding_) {
        // Enter holding phase
        RCLCPP_INFO(this->get_logger(), "Trajectory completed. Entering holding phase.");
        holding_ = true;

        KDL::Twist desired_vel;
        desired_vel.vel = KDL::Vector::Zero();
        desired_vel.rot = KDL::Vector::Zero(); // Use orientation error for rotational velocity

        KDL::Twist desired_acc;
        desired_acc.vel = KDL::Vector::Zero();
        desired_acc.rot = KDL::Vector::Zero();

        // Hold final position
        KDL::Frame desired_pose = current_pose;
        Eigen::VectorXd tau = controller_->idCntr(
            desired_pose,
            desired_vel,
            desired_acc,
            100.0, // Kp position
            50.0,  // Ko orientation
            2.0, // Kdp velocity
            1.0  // Kdo angular velocity
        );
        auto effort_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        effort_msg->data = std::vector<double>(tau.data(), tau.data() + tau.size());
        cmd_effort_pub_->publish(std::move(effort_msg));
    }
}
    std::shared_ptr<KDLRobot> robot_;
    std::unique_ptr<KDLController> controller_;
    std::unique_ptr<KDLPlanner> planner_;
    
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_efforts_;
    KDL::Frame init_cart_pose_;
    geometry_msgs::msg::PoseStamped marker_pose_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_effort_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool joint_states_received_ = false;
    bool marker_pose_received_ = false;
    bool holding_ = false;
    double t_;
    double trajectory_duration_;
    double acc_time_;
    double trajectory_radius_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircularTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}