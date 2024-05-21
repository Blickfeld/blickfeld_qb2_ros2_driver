/**
 * @file
 * @copyright Copyright (C) 2024, Blickfeld GmbH
 */

#pragma once

#include "qb2/point_cloud_reader.h"

#include "qb2_driver_status.h"
#include "qb2_ros2_type.h"

#include <rclcpp/rclcpp.hpp>

namespace blickfeld {
namespace ros_interop {
namespace qb2 {

class PointCloudStreamer : public PointCloudReader {
 public:
  using PointCloudReader::PointCloudReader;

  /**
   * @brief Read the Frame object from Qb2
   *
   * @return std::pair<CommunicationState, Qb2Frame> The state of communication and the new optional frame object
   */
  std::pair<CommunicationState, std::optional<Qb2Frame>> readFrame() final;

 private:

  /**
   * @brief Read the Frame object from Qb2.
   * The method doesn't cleanup the internal state after connection issues
   *
   * @return std::pair<CommunicationState, Qb2Frame> The state of communication and the new optional frame object
   */
  std::pair<CommunicationState, std::optional<Qb2Frame>> tryReadFrame();

  /**
   * @brief Attempts to open a point cloud stream if the qb2_channel is connected
   *
   * @return The state of communication with Qb2, if successful it should return
   * CommunicationState::SUCCESS_OPEN_STREAM
   */
  CommunicationState openPointCloudStream();

  /**
   * @brief Checks if the stream is open
   *
   * @return true
   * @return false
   */
  bool isStreamingPointCloud() const;

  /**
   * @brief disconnects from the server and cleans up all internal connection state
   */
  void disconnect();

  /// Unique pointers for Qb2 streams
  std::unique_ptr<grpc::ClientReader<Qb2PointCloudStreamResponse>> point_cloud_stream_ = nullptr;
};
}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
