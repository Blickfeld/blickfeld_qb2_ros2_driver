/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include "qb2_lidar_ros.h"
#include "qb2_ros2_type.h"

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace blickfeld {
namespace ros_interop {

/**
 * @class blickfeld::ros_interop::Qb2Driver
 * @brief Main class for the node to handle a live stream of point cloud from a single Qb2
 */
class Qb2Driver {
 public:
  /**
   * @brief Constructor, declares parameters
   * @param[in] options contains parameter environment
   */
  explicit Qb2Driver(rclcpp::NodeOptions options);

  /**
   * @brief Destructor
   */
  ~Qb2Driver();

  /**
   * @brief provide node base interface as required by ROS
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
    return node_->get_node_base_interface();
  }

 private:
  /**
   * @brief Spins the driver until ROS is not ok or shutdown is requested
   */
  void spinDriver();

  rclcpp::Node::SharedPtr node_;

  /// Flag to set if the ROS point cloud message should get the time stamp of the frame (measurement) or from
  /// ros::Time::now()
  bool use_measurement_timestamp_ = false;

  std::unique_ptr<Qb2LidarRos> qb2_;
  boost::asio::thread_pool spin_thread_{1};

  /// The Diagnostic Updater object to output device driver state
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::Heartbeat heartbeat_;
  std::shared_ptr<diagnostic_updater::TimeStampStatus> stamp_status_;
};

}  // namespace ros_interop
}  // namespace blickfeld
