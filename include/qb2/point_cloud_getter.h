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

class PointCloudGetter : public PointCloudReader {
 public:
  using PointCloudReader::PointCloudReader;

  /**
   * @brief Read the Frame object from Qb2
   *
   * @return std::pair<CommunicationState, Qb2Frame> The state of communication and the new optional frame object
   */
  std::pair<CommunicationState, std::optional<Qb2Frame>> readFrame() final;
};
}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
