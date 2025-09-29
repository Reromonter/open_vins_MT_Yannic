// created by Yannic 

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Your existing headers
#include "utils/print.h"
#include "utils/Recorder_ros2.h"

class PoseToFileNode : public rclcpp::Node {
public:
  PoseToFileNode() : Node("pose_to_file") {
    // Declare + get parameters
    verbosity_   = this->declare_parameter<std::string>("verbosity", "INFO");
    topic_       = this->declare_parameter<std::string>("topic", "");
    topic_type_  = this->declare_parameter<std::string>("topic_type", "");
    fileoutput_  = this->declare_parameter<std::string>("output", "");

    ov_core::Printer::setPrintLevel(verbosity_);

    PRINT_DEBUG("Done reading config values");
    PRINT_DEBUG(" - topic = %s", topic_.c_str());
    PRINT_DEBUG(" - topic_type = %s", topic_type_.c_str());
    PRINT_DEBUG(" - file = %s", fileoutput_.c_str());

    recorder_ = std::make_shared<ov_eval::Recorder>(fileoutput_);

    // Choose QoS (keep default reliable unless you know you need sensor_data)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Subscribe based on topic_type
    if (topic_type_ == "PoseWithCovarianceStamped") {
      sub_posecov_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          topic_, qos,
          std::bind(&ov_eval::Recorder::callback_posecovariance, recorder_.get(), std::placeholders::_1));
    } else if (topic_type_ == "PoseStamped") {
      sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          topic_, qos,
          std::bind(&ov_eval::Recorder::callback_pose, recorder_.get(), std::placeholders::_1));
    } else if (topic_type_ == "TransformStamped") {
      // NOTE: Most TF data comes on /tf as tf2_msgs::msg::TFMessage.
      // Use TransformStamped only if your topic actually publishes that type.
      sub_tf_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
          topic_, qos,
          std::bind(&ov_eval::Recorder::callback_transform, recorder_.get(), std::placeholders::_1));
    } else if (topic_type_ == "Odometry") {
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
          topic_, qos,
          std::bind(&ov_eval::Recorder::callback_odometry, recorder_.get(), std::placeholders::_1));
    } else {
      PRINT_ERROR("The specified topic type is not supported");
      PRINT_ERROR("topic_type = %s", topic_type_.c_str());
      PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
      rclcpp::shutdown();
      std::exit(EXIT_FAILURE);
    }
  }

private:
  std::string verbosity_, topic_, topic_type_, fileoutput_;
  std::shared_ptr<ov_eval::Recorder> recorder_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_posecov_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_tf_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToFileNode>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
