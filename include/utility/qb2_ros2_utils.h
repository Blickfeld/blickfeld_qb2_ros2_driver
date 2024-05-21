/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include "qb2_ros2_type.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <optional>
#include <string>

namespace blickfeld {
namespace ros_interop {

/**
 * @brief Stores the given @p value in the given @p point_cloud (using the appropriate indices)
 * @param[in, out] point_cloud the point cloud to store the value/field in
 * @param[in] point_index index of the point
 * @param[in] field_index index of the field within point_cloud.fields
 * @param[in] value the value to be assigned
 */
template <typename FieldT, typename ValueT>
__attribute__((always_inline)) inline void assignField(sensor_msgs::msg::PointCloud2& point_cloud, size_t point_index,
                                                       size_t field_index, const ValueT& value) {
  *reinterpret_cast<FieldT*>(
      &point_cloud.data[point_index * point_cloud.point_step + point_cloud.fields[field_index].offset]) =
      static_cast<FieldT>(value);
}

/**
 * @brief Add a field to a sensor_msgs::PointCloud2 message
 * @param[in, out] point_cloud the point cloud the fields gets added to
 * @param[in] name the name of the newly created field
 * @param[in] offset the offset defined by previously created fields
 * @param[in] datatype the datatype of the field
 */
template <typename FieldT>
__attribute__((always_inline)) inline void addPointCloudField(sensor_msgs::msg::PointCloud2& point_cloud,
                                                              const std::string& name, uint32_t offset,
                                                              uint8_t datatype) {
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.count = 1;
  field.datatype = datatype;
  field.offset = offset;
  point_cloud.fields.push_back(field);
  point_cloud.point_step += sizeof(FieldT);
}

/**
 * @brief Converts the frame into a sensor_msgs::PointCloud2 message
 * @param[in] frame the Qb2 frame/point cloud structure
 * @param[in] qb2 the Qb2Info object to contain info of the Qb2 that produced the point cloud
 * @param[in] timestamp if provided the timestamp to set in point cloud header, otherwise the timestamp of the 'frame'
 * will be used
 * @return the point cloud message
 */
std::unique_ptr<sensor_msgs::msg::PointCloud2> convertToPointCloudMsg(const Qb2Frame& frame, const Qb2Info& qb2,
                                                                      std::optional<rclcpp::Time> timestamp = {});

/**
 * @brief Convert the communication state into a string
 *
 * @param state
 * @return std::string
 */
std::string toString(CommunicationState state);

/**
 * @brief Check the grpc status for the case of authetication failure in the error msg
 *
 * @param status
 * @return true
 * @return false
 */
bool didAuthenticationFail(const grpc::Status& status);

}  // namespace ros_interop
}  // namespace blickfeld
