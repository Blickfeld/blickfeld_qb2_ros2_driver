#include "qb2/point_cloud_reader.h"

#include "utility/qb2_connection_utils.h"

namespace blickfeld {
namespace ros_interop {
namespace qb2 {
PointCloudReader::PointCloudReader(rclcpp::Node::SharedPtr node, const Qb2Info& qb2) : node_(node), qb2_(qb2) {}

PointCloudReader::~PointCloudReader() { cancel(); }

void PointCloudReader::cancel() {
  if (point_cloud_context_ != nullptr) {
    point_cloud_context_->TryCancel();
  }
}
}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
