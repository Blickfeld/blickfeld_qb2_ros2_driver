/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include "qb2/point_cloud_getter.h"
#include "qb2/point_cloud_reader.h"
#include "qb2/point_cloud_streamer.h"
#include "qb2/scan_pattern_watcher.h"
#include "qb2_driver_status.h"
#include "qb2_ros2_type.h"

#include <grpc++/client_context.h>
#include <boost/asio.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace blickfeld {
namespace ros_interop {
/**
 * @class blickfeld::ros_interop::Qb2LidarRos
 *
 * @brief Class wrapping the grpc communication with Qb2, providing functions to read and publish frames from Qb2
 */
class Qb2LidarRos {
 public:
  /**
   * @brief Constructor
   *
   * @param[in] node the driver node
   * @param[in] qb2 the Qb2Info object to contain info of the Qb2 to communicate with
   * @param[in, out]  diagnostic_updater the diagnostic updater for registering the diagnostic update function
   */
  Qb2LidarRos(rclcpp::Node::SharedPtr node, const Qb2Info& qb2, diagnostic_updater::Updater& diagnostic_updater,
              std::unique_ptr<qb2::PointCloudReader> point_cloud_reader);

  /**
   * @brief Destructor
   */
  ~Qb2LidarRos();

  /**
   * @brief Get the Qb2 info
   *
   * @return the Qb2 info that is used to communicate with
   */
  const Qb2Info& getQb2Info() const;

  /**
   * @brief Read one frame from Qb2
   *
   * @return optional frame from Qb2 if the read was successful
   */
  std::optional<Qb2Frame> readFrame();

  /**
   * @brief Publish the frame as a PointCloud2 msg
   * @param[in] frame the data from Qb2
   * @param[in] timestamp the timestamp to set in point cloud header
   */
  void publishFrame(const Qb2Frame& frame, rclcpp::Time timestamp);

  /**
   * @brief Stops all running threads and disconnects from the Qb2
   *
   */
  void shutdownThreads();

 private:
  /**
   * @brief Disconnect all connections to Qb2
   *
   */
  void disconnect();

  /**
   * @brief Check the connection to Qb2 if the last received frame is too old, cancel streams and disconnect from Qb2
   *
   */
  void checkForHangingStreams();

  /**
   * @brief Function to run on a separate thread to monitor the change to the scan pattern on the Qb2
   *
   */
  void watchScanPattern();

  /**
   * @brief Sleep in case the threads are running with a conditional variable that could interrupt the sleep
   *
   */
  void interruptableSleep();

  /**
   * @brief Disconnect from device and sleep if there is a failure in communication with Qb2
   *
   */
  void onCommunicationFailure();

  rclcpp::Node::SharedPtr node_;

  /// Qb2 info that we want to connect to
  Qb2Info qb2_;

  /// Collection of data that represent the status of the driver
  DriverStatus driver_status_;

  std::unique_ptr<qb2::PointCloudReader> point_cloud_reader_;
  std::unique_ptr<qb2::ScanPatternWatcher> scan_pattern_watcher_;

  /// ROS publisher for point cloud
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> point_cloud_publisher_raw_;
  std::shared_ptr<diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::PointCloud2>> point_cloud_publisher_;

  /// Threads used to watch the scan pattern and connection to Qb2
  boost::asio::thread_pool worker_thread_pool_{2};
  /// Variables for handling threads
  std::atomic<bool> is_running_{false};
  std::mutex is_running_mutex_;
  std::condition_variable is_running_condition_variable_;

  /// Parameters for frequency monitor
  double frequency_ = 1.0;
  const float frequency_tolerance_ = 0.1;
};

}  // namespace ros_interop
}  // namespace blickfeld
