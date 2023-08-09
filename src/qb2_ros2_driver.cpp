#include "qb2_ros2_driver.h"

#include <string>

namespace blickfeld {
namespace ros_interop {

Qb2Driver::Qb2Driver(rclcpp::NodeOptions options)
    : node_(std::make_shared<rclcpp::Node>("blickfeld_qb2_driver", options.use_intra_process_comms(true))) {
  /// set params
  use_measurement_timestamp_ = node_->declare_parameter<bool>("use_measurement_timestamp", use_measurement_timestamp_);
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "The 'use_measurement_timestamp' is set to: " << (use_measurement_timestamp_ ? "True" : "False"));
  auto fqdn = node_->declare_parameter<std::string>("fqdn", "");
  auto unix_socket = node_->declare_parameter<std::string>("unix_socket", "");
  auto frame_id = node_->declare_parameter<std::string>("frame_id", "lidar");
  auto point_cloud_topic = node_->declare_parameter<std::string>("point_cloud_topic", "~/point_cloud_out");

  /// figure out the host and method of connection
  if (unix_socket.empty() && fqdn.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "Neither unix socket nor fqdn were provided!");
    rclcpp::shutdown();
  }

  std::string host;
  bool use_socket_connection = true;
  /// logic to parse connection details, if both are provided, we'll prioritize unix socket
  if (unix_socket.empty() == false) {
    host = unix_socket;
    use_socket_connection = true;
  } else {
    host = fqdn;
    use_socket_connection = false;
  }

  qb2_ = std::make_unique<Qb2LidarRos>(node_, host, use_socket_connection, frame_id, point_cloud_topic);

  /// launch thread for execution
  is_running_ = true;
  spin_thread_ = std::make_shared<std::thread>(&Qb2Driver::spinDriver, this);
}

Qb2Driver::~Qb2Driver() {
  is_running_ = false;
  if (spin_thread_) {
    spin_thread_->join();
    spin_thread_ = nullptr;
  }
}

void Qb2Driver::spinDriver() {
  while (rclcpp::ok() == true && is_running_ == true) {
    Qb2Frame frame;
    auto success = qb2_->readFrame(frame);

    if (success) {
      qb2_->publishFrame(frame, use_measurement_timestamp_ ? rclcpp::Time(frame.timestamp()) : node_->now());
    }
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Stopped grabbing frames from Blickfeld Qb2 '" << qb2_->getHost() << "'!");
}

}  // namespace ros_interop
}  // namespace blickfeld

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blickfeld::ros_interop::Qb2Driver)
