/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include "qb2_lidar_ros.h"
#include "qb2_ros2_type.h"

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <string>
#include <vector>

namespace blickfeld {
namespace ros_interop {

/**
 * @class blickfeld::ros_interop::Qb2SnapshotDriver
 * @brief Main class for the node to handle snapshot point cloud from multiple Qb2
 */
class Qb2SnapshotDriver {
 public:
  /**
   * @brief Constructor, declares parameters
   * @param[in] options contains parameter environment
   */
  explicit Qb2SnapshotDriver(rclcpp::NodeOptions options);

  /**
   * @brief Destructor
   */
  ~Qb2SnapshotDriver();

  /**
   * @brief provide node base interface as required by ROS
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
    return node_->get_node_base_interface();
  }

 private:
  /**
   * @brief Creates qb2 and append it to the list of qb2s
   *
   * @param[in] fqdns the fqdn of qb2s to use for connection
   * @param[in] frame_ids the frame id for each qb2
   * @param[in] point_cloud_topics the topic for each qb2 to publish its point cloud
   * @param[in] snapshot_frame_rate the frame rate of snapshot
   * @param[in] intensity the flag to publish intensity
   * @param[in] point_id the flag to publish point_id
   */
  void setupQb2Fqdns(const std::vector<std::string>& fqdns, const std::vector<std::string>& fqdn_serial_numbers,
                     const std::vector<std::string>& fqdn_application_keys, const std::vector<std::string>& frame_ids,
                     const std::vector<std::string>& point_cloud_topics, float snapshot_frame_rate, bool intensity,
                     bool point_id);

  /**
   * @brief Creates qb2 and append it to the list of qb2s
   *
   * @param[in] system_unix_sockets the system unix socket of qb2s to use for connection
   * @param[in] core_processing_unix_sockets the core_processing unix socket of qb2s to use for connection
   * @param[in] frame_ids the frame id for each qb2
   * @param[in] point_cloud_topics the topic for each qb2 to publish its point cloud
   * @param[in] snapshot_frame_rate the frame rate of snapshot
   * @param[in] intensity the flag to publish intensity
   * @param[in] point_id the flag to publish point_id
   */
  void setupQb2UnixSockets(const std::vector<std::string>& system_unix_sockets,
                           const std::vector<std::string>& core_processing_unix_sockets,
                           const std::vector<std::string>& frame_ids,
                           const std::vector<std::string>& point_cloud_topics, float snapshot_frame_rate,
                           bool intensity, bool point_id);

  /**
   * @brief Get one frame form each qb2 and convert it to PointCloud2 and publish it
   */
  void snapshot();

  /**
   * @brief Callback to handle running snapshot on a separate thread and return if the previous snapshot is not finished
   *
   * @return true if we could trigger a snapshot, false if one was already in progress
   */
  bool snapshotTriggerCallback();

  rclcpp::Node::SharedPtr node_;

  /// Flag to set if the ROS point cloud message should get the time stamp of the frame (measurement) or from
  /// ros::Time::now()
  bool use_measurement_timestamp_ = false;

  /// Max number of tries to snapshot a point cloud from each qb2
  uint8_t max_retries_ = 3;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_snapshot_service_;

  std::vector<std::unique_ptr<Qb2LidarRos>> qb2s_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;

  /// snapshot cannot be faster than every 10 seconds (max frame_rate of 0.1)
  static constexpr double max_allowed_snapshot_frame_rate_ = 0.1;
  /// setting frame_rate to 0 means no timer based snapshot
  static constexpr double min_allowed_snapshot_frame_rate_ = 0;

  /// flag for running the device in snapshot mode
  static constexpr bool snapshot_mode_ = true;

  /// Thread used to get the snapshot from Qb2s
  boost::asio::thread_pool snapshot_thread_{1};
  std::atomic<bool> snapshot_is_running_{false};

  /// The Diagnostic Updater object to output device driver state
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::Heartbeat heartbeat_;
  std::shared_ptr<diagnostic_updater::TimeStampStatus> stamp_status_;
};

}  // namespace ros_interop
}  // namespace blickfeld
