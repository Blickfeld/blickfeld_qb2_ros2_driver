#include "qb2_ros2_utils.h"

#include <grpc++/client_context.h>

namespace blickfeld {
namespace ros_interop {

std::unique_ptr<sensor_msgs::msg::PointCloud2> convertToPointCloudMsg(const Qb2Frame& frame,
                                                                      const std::string& frame_id,
                                                                      const PointFields& point_fields,
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

  if (point_fields.intensity == true) {
    /// TODO: intensity should be converted to UINT16 (as it is also published by Qb2) Sticking to the Blickfeld
    /// PointCloud2 format here, which uses uint32
    addPointCloudField<uint32_t>(std::ref(*point_cloud), "intensity", point_cloud->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
  }
  if (point_fields.point_id == true) {
    addPointCloudField<uint32_t>(std::ref(*point_cloud), "point_id", point_cloud->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
  }

  float* cartesian = (float*)frame.binary().cartesian().data();
  uint16_t* photon_count = (uint16_t*)frame.binary().photon_count().data();
  uint32_t* direction_id = (uint32_t*)frame.binary().direction_id().data();

  /// reserve memory
  point_cloud->data.resize(number_of_points * point_cloud->point_step);

  /// set point cloud message data
  point_cloud->header.frame_id = frame_id;
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
    if (point_fields.intensity == true) assignField<uint32_t>(std::ref(*point_cloud), i, 3, photon_count[0]);
    /// point_id
    if (point_fields.point_id == true) assignField<uint32_t>(std::ref(*point_cloud), i, 4, direction_id[0]);

    // advance pointers
    cartesian = cartesian + 3;
    photon_count++;
    direction_id++;
  }
  return point_cloud;
}

std::shared_ptr<grpc::Channel> connectToUnixSocket(const std::string& unix_socket_name,
                                                   std::chrono::duration<unsigned int> timeout) {
  const std::string unix_socket_target = "unix://" + unix_socket_name;
  grpc::ClientContext context;
  std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel(unix_socket_target, grpc::InsecureChannelCredentials());

  auto deadline = std::chrono::system_clock::now() + std::chrono::duration_cast<std::chrono::seconds>(timeout);
  // Wait for connection
  grpc_connectivity_state state;
  while ((state = channel->GetState(true)) != GRPC_CHANNEL_READY && state != GRPC_CHANNEL_TRANSIENT_FAILURE) {
    if (!channel->WaitForStateChange(state, deadline)) {
      throw std::runtime_error("Connection to Qb2 device '" + unix_socket_target + "' failed. Deadline exceeded.");
    }
  }
  if (state != grpc_connectivity_state::GRPC_CHANNEL_READY) {
    throw std::runtime_error("Connection to Qb2 device '" + unix_socket_target + "' failed. Network failure.");
  }
  return channel;
}

}  // namespace ros_interop
}  // namespace blickfeld
