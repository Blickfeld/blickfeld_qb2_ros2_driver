/**
 * @file
 * @copyright Copyright (C) 2024, Blickfeld GmbH
 */

#pragma once

#include "qb2_ros2_type.h"

#include <blickfeld/base/grpc_defs.h>

#include <grpc++/create_channel.h>
#include <rclcpp/rclcpp.hpp>

namespace blickfeld {
namespace ros_interop {
namespace qb2 {

/**
 * @brief Establishes a grpc channel to a unix socket
 * NOTE: The method throws a runtime error if the connection cannot be established
 * @param[in] unix_socket_name the path to the unix socket
 * @param[in] timeout longest duration to wait for a valid connection
 * @return the established grpc channel
 */
std::shared_ptr<grpc::Channel> connectToUnixSocket(
    const std::string& unix_socket_name,
    std::chrono::duration<unsigned int> timeout = base::GRPC_DEFAULT_CONNECTION_TIMEOUT);

/**
 * @brief Attempts to create a new grpc channel
 *
 * @param qb2 Qb2Info object containing the info regarding the device
 * @param connection_target The target that we want to connect to in case of unix socket we can connect to system or
 * core_processing
 * @param logger ROS2 logger to log the state of the function
 * @return Qb2ChannelPtr if connection was successful it will return the grpc channel otherwise a nullptr
 */
Qb2ChannelPtr connect(const Qb2Info& qb2, const ConnectionTarget& connection_target, const rclcpp::Logger& logger);

/**
 * @brief Sets the qb2_channel pointer to nullptr
 *
 * @param qb2_channel The gRPC channel connection to Qb2
 * @param qb2 Qb2Info object containing the info regarding the device
 * @param logger ROS2 logger to log the state of the function
 */
void disconnect(Qb2ChannelPtr& qb2_channel, const Qb2Info& qb2, const rclcpp::Logger& logger);

/**
 * @brief Checks the status of the gRPC connection channel to Qb2
 *
 * @param qb2_channel The gRPC connection channel
 * @return true
 * @return false
 */
bool isConnected(const Qb2ChannelPtr& qb2_channel);

/**
 * @brief Check if there is a failure with communication with Qb2
 *
 * @return true
 * @return false
 */
bool hasQb2CommunicationFailed(CommunicationState communication_state);

}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
