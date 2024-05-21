#include "qb2_lidar_ros.h"

#include "utility/qb2_ros2_utils.h"

#include <blickfeld/base/grpc_defs.h>

#include <grpc++/create_channel.h>
#include <diagnostic_updater/publisher.hpp>

#include <optional>
#include <utility>

namespace blickfeld {
namespace ros_interop {

Qb2LidarRos::Qb2LidarRos(rclcpp::Node::SharedPtr node, const Qb2Info& qb2,
                         diagnostic_updater::Updater& diagnostic_updater,
                         std::unique_ptr<qb2::PointCloudReader> point_cloud_reader)
    : node_(node), qb2_(qb2), driver_status_(qb2), point_cloud_reader_(std::move(point_cloud_reader)) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'host' is set to: " << qb2_.hostName());
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'frame_id' is set to: " << qb2_.point_cloud_info.frame_id);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'point_cloud_topic' is set to: " << qb2_.point_cloud_info.topic);

  /// diagnostic updater
  diagnostic_updater.add("Diagnostic message - " + qb2_.hostName(), &driver_status_, &DriverStatus::updateDiagnostic);

  scan_pattern_watcher_ = std::make_unique<qb2::ScanPatternWatcher>(node, qb2);

  /// HINT: Snapshot mode: The window size contains one snapshot plus a 10 second buffer for establishing the connection
  /// with the devices. Live mode: The window size is 5 seconds
  const int window_size = qb2_.snapshot.mode ? (1 / qb2_.snapshot.frame_rate) + 10 : 5;
  diagnostic_updater::FrequencyStatusParam frequency_status(&frequency_, &frequency_, frequency_tolerance_,
                                                            window_size);
  diagnostic_updater::TimeStampStatusParam timestamp_delay;

  /// create the ros publisher
  point_cloud_publisher_raw_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(qb2_.point_cloud_info.topic, 1);
  point_cloud_publisher_ = std::make_shared<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::PointCloud2>>(
      point_cloud_publisher_raw_, diagnostic_updater, frequency_status, timestamp_delay);

  is_running_ = true;
  /// thread to make sure connection is correctly established
  boost::asio::post(worker_thread_pool_, [&]() { this->checkForHangingStreams(); });
  /// thread for running the scan pattern watch
  boost::asio::post(worker_thread_pool_, [&]() { this->watchScanPattern(); });
}

Qb2LidarRos::~Qb2LidarRos() {
  shutdownThreads();

  worker_thread_pool_.stop();
  worker_thread_pool_.join();
}

void Qb2LidarRos::shutdownThreads() {
  /// Call disconnect to stop all streams and communication with Qb2
  disconnect();
  {
    std::lock_guard<std::mutex> guard(is_running_mutex_);
    is_running_ = false;
  }
  is_running_condition_variable_.notify_all();
}

const Qb2Info& Qb2LidarRos::getQb2Info() const { return qb2_; }

std::optional<Qb2Frame> Qb2LidarRos::readFrame() {
  std::optional<Qb2Frame> frame;
  auto request_api_timestamp = std::chrono::high_resolution_clock::now();
  CommunicationState communication_state = CommunicationState::NOT_DEFINED;
  std::tie(communication_state, frame) = point_cloud_reader_->readFrame();
  auto received_api_timestamp = std::chrono::high_resolution_clock::now();

  uint64_t frame_id = frame.has_value() ? frame.value().id() : 0;
  driver_status_.updateRuntimeStatus(frame_id, communication_state, request_api_timestamp, received_api_timestamp);

  if (qb2::hasQb2CommunicationFailed(communication_state) == true) {
    onCommunicationFailure();
  }
  return frame;
}

void Qb2LidarRos::publishFrame(const Qb2Frame& frame, rclcpp::Time timestamp) {
  auto point_cloud = convertToPointCloudMsg(frame, qb2_, timestamp);
  point_cloud_publisher_->publish(std::move(point_cloud));

  driver_status_.onPublishingFrame();
}

void Qb2LidarRos::disconnect() {
  /// HINT: these cancel calls are needed to interrupt the streams and tell the server that we are closing the
  /// connection
  point_cloud_reader_->cancel();
  scan_pattern_watcher_->cancel();

  driver_status_.disconnect();
}

void Qb2LidarRos::checkForHangingStreams() {
  while (is_running_ == true) {
    if (driver_status_.isFrameTooOld() == true) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Last frame from Qb2: " << qb2_.hostName() << " is too old, disconnecting!");
      disconnect();
    }
    interruptableSleep();
  }
}

void Qb2LidarRos::watchScanPattern() {
  while (is_running_ == true) {
    std::optional<Qb2ScanPattern> scan_pattern;
    CommunicationState communication_state = CommunicationState::NOT_DEFINED;
    std::tie(communication_state, scan_pattern) = scan_pattern_watcher_->watchScanPattern();

    driver_status_.updateScanPattern(scan_pattern, communication_state);

    if (qb2::hasQb2CommunicationFailed(communication_state) == false) {
      frequency_ = driver_status_.getTargetFrameRate();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Scan Pattern changed!");
      RCLCPP_INFO_STREAM(node_->get_logger(), "Target frame rate: " << frequency_);
    } else {
      onCommunicationFailure();
    }
  }
}

void Qb2LidarRos::interruptableSleep() {
  std::unique_lock<std::mutex> lock(is_running_mutex_);
  if (is_running_ == true) {
    is_running_condition_variable_.wait_for(lock, std::chrono::seconds(base::GRPC_DEFAULT_CONNECTION_TIMEOUT));
  }
}

void Qb2LidarRos::onCommunicationFailure() {
  disconnect();
  interruptableSleep();
}

}  // namespace ros_interop
}  // namespace blickfeld
