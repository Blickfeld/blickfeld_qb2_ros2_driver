#include "qb2_ros2_driver.h"

#include <rclcpp/exceptions.hpp>

#include <string>

namespace blickfeld {
namespace ros_interop {

Qb2Driver::Qb2Driver(rclcpp::NodeOptions options)
    : node_(std::make_shared<rclcpp::Node>("blickfeld_qb2_driver", options.use_intra_process_comms(true))),
      diagnostic_updater_(node_) {
  /// set parameters
  std::string fqdn = node_->declare_parameter<std::string>("fqdn", "");
  std::string serial_number = node_->declare_parameter<std::string>("serial_number", "");
  std::string application_key = node_->declare_parameter<std::string>("application_key", "");

  std::string system_unix_socket = node_->declare_parameter<std::string>("system_unix_socket", "");

  std::string core_processing_unix_socket = node_->declare_parameter<std::string>("core_processing_unix_socket", "");
  std::string frame_id = node_->declare_parameter<std::string>("frame_id", "lidar");
  std::string point_cloud_topic = node_->declare_parameter<std::string>("point_cloud_topic", "~/point_cloud_out");

  use_measurement_timestamp_ = node_->declare_parameter<bool>("use_measurement_timestamp", use_measurement_timestamp_);
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "The 'use_measurement_timestamp' is set to: " << (use_measurement_timestamp_ ? "True" : "False"));

  bool intensity = node_->declare_parameter<bool>("publish_intensity", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'publish_intensity' is set to: " << (intensity ? "True" : "False"));

  bool point_id = node_->declare_parameter<bool>("publish_point_id", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'publish_point_id' is set to: " << (point_id ? "True" : "False"));

  /// figure out the host and method of connection
  if ((system_unix_socket.empty() || core_processing_unix_socket.empty()) && fqdn.empty()) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Neither unix socket nor fqdn were provided!");
    rclcpp::shutdown();
  }

  Host host;
  /// logic to parse connection details, if both are provided, we'll prioritize unix socket
  if ((system_unix_socket.empty() && core_processing_unix_socket.empty()) == false) {
    host.system_socket = system_unix_socket;
    host.core_processing_socket = core_processing_unix_socket;
    host.connection_type = ConnectionType::UNIX_SOCKET;
  } else {
    host.fqdn = fqdn;
    host.connection_type = ConnectionType::FQDN;
  }
  host.serial_number = serial_number;
  host.application_key = application_key;

  const bool snapshot_mode = false;
  const float snapshot_frame_rate = 0.f;
  Qb2Info qb2_info{host, PointCloudInfo{frame_id, point_cloud_topic, intensity, point_id},
                   Snapshot{snapshot_mode, snapshot_frame_rate}};

  /// set the name of the driver as the hardwareID of the updater
  diagnostic_updater_.setHardwareID("Blickfeld Qb2 Driver");
  diagnostic_updater_.add(heartbeat_);
  stamp_status_ = std::make_shared<diagnostic_updater::TimeStampStatus>(diagnostic_updater::TimeStampStatusParam(),
                                                                        "Read frame time");
  diagnostic_updater_.add(*stamp_status_);

  auto point_cloud_reader = std::make_unique<qb2::PointCloudStreamer>(node_, qb2_info);
  qb2_ = std::make_unique<Qb2LidarRos>(node_, qb2_info, diagnostic_updater_, std::move(point_cloud_reader));

  /// launch thread for execution
  boost::asio::post(spin_thread_, [&]() { this->spinDriver(); });
}

Qb2Driver::~Qb2Driver() {
  qb2_->shutdownThreads();

  spin_thread_.stop();
  spin_thread_.join();
}

void Qb2Driver::spinDriver() {
  while (rclcpp::ok() == true) {
    rclcpp::Time read_frame_ts = node_->now();
    std::optional<Qb2Frame> frame = qb2_->readFrame();
    stamp_status_->tick(read_frame_ts);

    if (frame.has_value() == true) {
      qb2_->publishFrame(frame.value(),
                         use_measurement_timestamp_ ? rclcpp::Time(frame.value().timestamp()) : node_->now());
    }
  }
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Stopped grabbing frames from Blickfeld Qb2: " << qb2_->getQb2Info().hostName() << "!");
}

}  // namespace ros_interop
}  // namespace blickfeld

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blickfeld::ros_interop::Qb2Driver)
