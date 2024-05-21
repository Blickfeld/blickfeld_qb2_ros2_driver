/**
 * @file
 * @copyright Copyright (C) 2024, Blickfeld GmbH
 */

#pragma once

#include "qb2_driver_status.h"
#include "qb2_ros2_type.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <utility>

namespace blickfeld {
namespace ros_interop {
namespace qb2 {

class PointCloudReader {
 public:
  PointCloudReader(rclcpp::Node::SharedPtr node, const Qb2Info& qb2);

  virtual ~PointCloudReader();

  /**
   * @brief Read the Frame object from Qb2
   *
   * @return std::pair<CommunicationState, Qb2Frame> The state of communication and the new optional frame object
   */
  virtual std::pair<CommunicationState, std::optional<Qb2Frame>> readFrame() = 0;

  /**
   * @brief Try to cancel the running getter
   *
   */
  virtual void cancel();

 protected:
  rclcpp::Node::SharedPtr node_;

  /// Device info that we want to connect to
  Qb2Info qb2_;

  /// Connection channel to the Qb2 gRPC server
  std::shared_ptr<grpc::Channel> qb2_channel_;

  /// The gRPC client context for Qb2
  std::unique_ptr<grpc::ClientContext> point_cloud_context_ = nullptr;
};
}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
