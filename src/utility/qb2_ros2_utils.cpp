#include "utility/qb2_ros2_utils.h"

#include <grpc++/client_context.h>

namespace blickfeld {
namespace ros_interop {

std::unique_ptr<sensor_msgs::msg::PointCloud2> convertToPointCloudMsg(const Qb2Frame& frame, const Qb2Info& qb2,
                                                                      std::optional<rclcpp::Time> timestamp) {
  auto point_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();

  /// if timestamp is passed use that otherwise get the time from frame
  if (timestamp) {
    point_cloud->header.stamp = *timestamp;
  } else {
    point_cloud->header.stamp = rclcpp::Time(frame.timestamp());
  }

  const auto number_of_points = frame.binary().length();

  /// add fields to PointCloud2 msg
  addPointCloudField<float>(std::ref(*point_cloud), "x", point_cloud->point_step,
                            sensor_msgs::msg::PointField::FLOAT32);
  addPointCloudField<float>(std::ref(*point_cloud), "y", point_cloud->point_step,
                            sensor_msgs::msg::PointField::FLOAT32);
  addPointCloudField<float>(std::ref(*point_cloud), "z", point_cloud->point_step,
                            sensor_msgs::msg::PointField::FLOAT32);

  if (qb2.point_cloud_info.intensity == true) {
    /// HINT: To stick to Blickfeld PointCloud2 format the intensity is published as UINT32 and not as UINT16 that is
    /// published by Qb2 as photon count
    addPointCloudField<uint32_t>(std::ref(*point_cloud), "intensity", point_cloud->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
  }
  if (qb2.point_cloud_info.point_id == true) {
    addPointCloudField<uint32_t>(std::ref(*point_cloud), "point_id", point_cloud->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
  }

  float* cartesian = (float*)frame.binary().cartesian().data();
  uint16_t* photon_count = (uint16_t*)frame.binary().photon_count().data();
  uint32_t* direction_id = (uint32_t*)frame.binary().direction_id().data();

  /// reserve memory
  point_cloud->data.resize(number_of_points * point_cloud->point_step);

  /// set point cloud message data
  point_cloud->header.frame_id = qb2.point_cloud_info.frame_id;
  point_cloud->is_dense = false;
  point_cloud->height = 1;
  point_cloud->width = number_of_points;
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;

  /// copy the data
  for (unsigned int i = 0; i < number_of_points; i++) {
    /// cartesian (X, Y, Z)
    assignField<float>(std::ref(*point_cloud), i, 0, cartesian[0]);
    assignField<float>(std::ref(*point_cloud), i, 1, cartesian[1]);
    assignField<float>(std::ref(*point_cloud), i, 2, cartesian[2]);
    /// intensity
    if (qb2.point_cloud_info.intensity == true) assignField<uint32_t>(std::ref(*point_cloud), i, 3, photon_count[0]);
    /// point_id
    if (qb2.point_cloud_info.point_id == true) assignField<uint32_t>(std::ref(*point_cloud), i, 4, direction_id[0]);

    // advance pointers
    cartesian = cartesian + 3;
    photon_count++;
    direction_id++;
  }
  return point_cloud;
}

std::string toString(CommunicationState state) {
  switch (state) {
    case CommunicationState::NOT_DEFINED:
      return "NOT_DEFINED";
      break;
    case CommunicationState::SUCCESS_READ:
      return "SUCCESS_READ";
      break;
    case CommunicationState::SUCCESS_OPEN_STREAM:
      return "SUCCESS_OPEN_STREAM";
      break;
    case CommunicationState::FAIL_READ:
      return "FAIL_READ";
      break;
    case CommunicationState::FAIL_NO_CONNECTION:
      return "FAIL_NO_CONNECTION";
      break;
    case CommunicationState::FAIL_EXCEPTION:
      return "FAIL_EXCEPTION";
      break;
    case CommunicationState::FAIL_AUTHENTICATION:
      return "FAIL_AUTHENTICATION";
      break;
    case CommunicationState::DISCONNECTED:
      return "DISCONNECTED";
      break;

    default:
      return "UNKNOWN";
      break;
  }
}

bool didAuthenticationFail(const grpc::Status& status) {
  return (std::string(status.error_message()).find("Authentication failed") != std::string::npos) ||
         (std::string(status.error_message()).find("Token renewal failed") != std::string::npos);
}

}  // namespace ros_interop
}  // namespace blickfeld
