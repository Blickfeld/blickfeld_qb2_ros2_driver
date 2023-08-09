/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include "qb2_lidar_ros.h"
#include "qb2_ros2_type.h"

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
   * @param[in] fqdns the fqn of qb2 to use for connection
   * @param[in] frame_ids the frame id for each qb2
   * @param[in] point_cloud_topics the topic for each qb2 to publish its point cloud
   */
  void setupQb2sWithFqdns(const std::vector<std::string>& fqdns, const std::vector<std::string>& frame_ids,
                          const std::vector<std::string>& point_cloud_topics);

  /**
   * @brief
   *
   * @param[in] unix_sockets the unix socket of qb2 to use for connection
   * @param[in] frame_ids the frame id for each qb2
   * @param[in] point_cloud_topics the topic for each qb2 to publish its point cloud
   */
  void setupQb2sWithUnixSockets(const std::vector<std::string>& unix_sockets, const std::vector<std::string>& frame_ids,
                                const std::vector<std::string>& point_cloud_topics);

  /**
   * @brief Get one frame form each qb2 and convert it to PointCloud2 and publish it
   *
   * @return true if successful
   */
  bool snapshot();

  rclcpp::Node::SharedPtr node_;

  /// Flag to set if the ROS point cloud message should get the time stamp of the frame (measurement) or from
  /// ros::Time::now()
  bool use_measurement_timestamp_ = false;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_snapshot_service_;

  std::vector<std::unique_ptr<Qb2LidarRos>> qb2s_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;

  /// snapshot cannot be faster than every 10 seconds (max frequency of 0.1)
  static constexpr double max_allowed_snapshot_frequency_ = 0.1;
  /// setting frequency to 0 means no timer based snapshot
  static constexpr double min_allowed_snapshot_frequency_ = 0;

  /// flag for running the device in snapshot mode
  static constexpr bool snapshot_mode_ = true;
};

}  // namespace ros_interop
}  // namespace blickfeld
