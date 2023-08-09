/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include <blickfeld/core_processing/data/frame.pb.h>

namespace blickfeld {
namespace ros_interop {

using Qb2Frame = core_processing::data::Frame;

/// Point fields for the point cloud in addition to (x,y,z)
struct PointFields {
  bool intensity = true;
  bool point_id = true;
};

}  // namespace ros_interop
}  // namespace blickfeld
