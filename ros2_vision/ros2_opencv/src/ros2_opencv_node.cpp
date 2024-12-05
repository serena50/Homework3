#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class ContourDetectorNode : public rclcpp::Node {
public:
  ContourDetectorNode() : Node("contour_detector_node") {
    // Create publishers and subscribers
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera", 10,
        std::bind(&ContourDetectorNode::image_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image", 10);
    RCLCPP_INFO(this->get_logger(), "Contour detector node initialized");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      // Convert ROS Image to OpenCV Mat
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat frame = cv_ptr->image.clone();
      
      // Convert to grayscale
      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

      // Apply threshold to get binary image
      cv::Mat binary;
      // Binary threshold optimized for blue sphere on gray background
      cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);

      // Find contours
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // Process each contour
      for(const auto& contour : contours) {
        // Calculate area of the contour
        double area = cv::contourArea(contour);
        
        // Filter small contours
        // Filter by area based on sphere radius and camera FOV
        if(area > 10000 && area < 60000) {  // Area range based on sphere size
          // Find the minimum enclosing circle
          cv::Point2f center;
          float radius;
          cv::minEnclosingCircle(contour, center, radius);

          // Calculate circularity
          double perimeter = cv::arcLength(contour, true);
          double circularity = 4 * CV_PI * area / (perimeter * perimeter);

          // If shape is approximately circular
          // Higher circularity threshold since we know it's a perfect sphere
          if(circularity > 0.85) {
            // Draw the circle
            cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 2);
            
            // Draw center point
            cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), -1);
            
            // Add text with coordinates
            std::string coords = "(" + std::to_string(int(center.x)) + "," + 
                               std::to_string(int(center.y)) + ")";
            cv::putText(frame, coords, 
                       cv::Point(center.x + 10, center.y), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.5,
                       cv::Scalar(0, 255, 0), 2);

            RCLCPP_INFO(this->get_logger(), 
                       "Circle detected at (%f, %f) with radius %f", 
                       center.x, center.y, radius);
          }
        }
      }

      // Publish result
      sensor_msgs::msg::Image::SharedPtr out_msg = 
          cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      publisher_->publish(*out_msg);

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContourDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}