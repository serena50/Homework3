#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <memory>
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

using namespace std::chrono_literals;

class KDLVisionControl : public rclcpp::Node
{
public:
    KDLVisionControl() : Node("kdl_vision_control"), restart_attempts_(0)
    {
        // Declare only task parameter
        this->declare_parameter("task", "positioning");
        task_ = this->get_parameter("task").as_string();

        // Control parameters
        position_gain_ = 0.7;
        orientation_gain_ = 0.25;
        desired_distance_ = 0.25;
        look_at_point_gain_ = 1.0;  // Configurable gain for look-at-point task

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&KDLVisionControl::joint_state_callback, this, std::placeholders::_1));

        marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10,
            std::bind(&KDLVisionControl::marker_pose_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);

        // Initialize robot
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

        joint_values_.resize(nj);
        joint_velocities_.resize(nj);
        joint_efforts_.resize(nj);
        initial_joint_position_.resize(nj);

        // Define explicit camera-end effector transformation
        KDL::Frame ee_T_cam;
        ee_T_cam.M = KDL::Rotation::RotY(M_PI/2) * KDL::Rotation::RotZ(-M_PI/2);
        ee_T_cam.p = KDL::Vector(0, 0, 0.01);  // 2.5cm offset along z
        robot_->addEE(ee_T_cam);

        timer_ = this->create_wall_timer(
            20ms, std::bind(&KDLVisionControl::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Vision control node initialized with task: %s", task_.c_str());
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Check if received message matches expected joint count
        if (msg->position.size() != joint_values_.data.size()) {
            if (restart_attempts_ < 10000) {
                restart_attempts_++;
                RCLCPP_WARN(this->get_logger(), "Connection error. Attempting to reload the node. Attempt %d of 10.", restart_attempts_);
                
                // Cleanup and reinitialize
                joint_states_received_ = false;
                marker_pose_received_ = false;
                initial_position_stored_ = false;

                // Recreate subscriptions
                joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/joint_states", 10,
                    std::bind(&KDLVisionControl::joint_state_callback, this, std::placeholders::_1));

                marker_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/aruco_single/pose", 10,
                    std::bind(&KDLVisionControl::marker_pose_callback, this, std::placeholders::_1));
            }
            return;
        }

        for (size_t i = 0; i < msg->position.size(); i++) {
            joint_values_.data[i] = msg->position[i];
            joint_velocities_.data[i] = msg->velocity[i];
            joint_efforts_.data[i] = msg->effort[i];
        }
        
        if (!initial_position_stored_) {
            initial_joint_position_ = joint_values_;
            initial_position_stored_ = true;
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
        if (!joint_states_received_ || !marker_pose_received_) {
            return;
        }

        // Validate joint state data before using
        if (joint_values_.data.size() != 7 || joint_velocities_.data.size() != 7) {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint state data");
            return;
        }

        // Update robot state
        robot_->update(toStdVector(joint_values_.data), 
                      toStdVector(joint_velocities_.data));
        
        // Get current camera pose
        KDL::Frame current_camera_pose = robot_->getEEFrame();
        
        Eigen::VectorXd v_cmd;
        
        if (task_ == "positioning") {
            KDL::Frame marker_pose;
            marker_pose.p = KDL::Vector(
                marker_pose_.pose.position.x,
                marker_pose_.pose.position.y,
                marker_pose_.pose.position.z
            );
            marker_pose.M = KDL::Rotation::Quaternion(
                marker_pose_.pose.orientation.x,
                marker_pose_.pose.orientation.y,
                marker_pose_.pose.orientation.z,
                marker_pose_.pose.orientation.w
            );

            double desired_x_distance = 0.45;
            double desired_z_distance = 0.05;
            double desired_y_distance = 0.05;

            double x_error = marker_pose.p.z() - desired_x_distance;
            double z_error = -marker_pose.p.y() - desired_z_distance;
            double y_error = marker_pose.p.x() - desired_y_distance;

            Vector6d e_dot_cmd = Vector6d::Zero();
            e_dot_cmd[0] = -position_gain_ * x_error;
            e_dot_cmd[1] = position_gain_ * y_error;
            e_dot_cmd[2] = position_gain_ * z_error;

            KDL::Jacobian J_cam = robot_->getEEJacobian();
            v_cmd = pseudoinverse(J_cam.data) * e_dot_cmd;

            // Debug info
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "X-axis error: %.3f, Z-axis error: %.3f, Y-axis error: %.3f\nVelocity Command: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                x_error, z_error, y_error, v_cmd(0), v_cmd(1), v_cmd(2), v_cmd(3), v_cmd(4), v_cmd(5), v_cmd(6)
            );
        }
        else if (task_ == "look-at-point") {
            // Get marker position in camera frame
            KDL::Frame marker_pose;
            marker_pose.p = KDL::Vector(
                marker_pose_.pose.position.x,
                marker_pose_.pose.position.y,
                marker_pose_.pose.position.z
            );
            marker_pose.M = KDL::Rotation::Quaternion(
                marker_pose_.pose.orientation.x,
                marker_pose_.pose.orientation.y,
                marker_pose_.pose.orientation.z,
                marker_pose_.pose.orientation.w
            );

            // Computing s (eq.1)
            Eigen::Vector3d c_P_o = toEigen(marker_pose.p);
            double c_P_o_norm = c_P_o.norm();
            Eigen::Vector3d s = c_P_o / c_P_o_norm;

            // R_c is the current camera rotation matrix
            Eigen::Matrix3d R_c = toEigen(robot_->getEEFrame().M);

            // Construct R (eq.2)
            Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
            R.block<3,3>(0,0) = R_c;
            R.block<3,3>(3,3) = R_c;

            // Computing L(s) (eq.2)
            Eigen::Matrix<double, 3, 6> L = Eigen::Matrix<double, 3, 6>::Zero();
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d ssT = s * s.transpose();
            L.block<3,3>(0,0) = -1.0/c_P_o_norm * (I - ssT);
            L.block<3,3>(0,3) = skew(s);
            L = L * R.transpose();  

            // Get camera Jacobian
            KDL::Jacobian J_cam = robot_->getEEJacobian();

            // Computing LJ and its pseudoinverse
            Eigen::MatrixXd LJ = L * J_cam.data;
            Eigen::MatrixXd LJ_pinv = LJ.completeOrthogonalDecomposition().pseudoInverse();

            // Computing null space projector
            Eigen::MatrixXd N = Eigen::MatrixXd::Identity(7,7) - LJ_pinv * LJ;

            // Desired s
            Eigen::Vector3d s_d(0, 0, 1);  // Looking straight ahead

            // Task secondary: try to maintain initial joint configuration
            Eigen::VectorXd q_dot_0 = 0.1 * (initial_joint_position_.data - joint_values_.data);

            // Final control law
            v_cmd = look_at_point_gain_ * LJ_pinv * s_d + N * q_dot_0;

            // Debug info
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "s: [%.3f, %.3f, %.3f], s_d: [%.3f, %.3f, %.3f], norm_v: %.3f\nVelocity Command: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                s.x(), s.y(), s.z(),
                s_d.x(), s_d.y(), s_d.z(),
                v_cmd.norm(), v_cmd(0), v_cmd(1), v_cmd(2), v_cmd(3), v_cmd(4), v_cmd(5), v_cmd(6)
            );
        }

        // Apply velocity limits
        double max_vel = 0.5;  // rad/s
        if (v_cmd.norm() > max_vel) {
            v_cmd *= max_vel / v_cmd.norm();
        }

        // Publish velocity command
        auto vel_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        vel_msg->data = std::vector<double>(v_cmd.data(), v_cmd.data() + v_cmd.size());
        cmd_vel_pub_->publish(std::move(vel_msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<KDLRobot> robot_;
    
    KDL::JntArray joint_values_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_efforts_;
    KDL::JntArray initial_joint_position_;
    geometry_msgs::msg::PoseStamped marker_pose_;

    bool joint_states_received_ = false;
    bool marker_pose_received_ = false;
    bool initial_position_stored_ = false;
    std::string task_;

    // Control parameters
    double position_gain_;
    double orientation_gain_;
    double desired_distance_;
    double look_at_point_gain_;

    // New member to track restart attempts
    int restart_attempts_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KDLVisionControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}