#include "utility/qb2_connection_utils.h"

#include <blickfeld/hardware/client.h>

namespace blickfeld {
namespace ros_interop {
namespace qb2 {

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

Qb2ChannelPtr connect(const Qb2Info& qb2, const ConnectionTarget& connection_target, const rclcpp::Logger& logger) {
  /// create the correct connection method
  std::function<Qb2ChannelPtr()> connect_to_qb2;
  if (qb2.host.connection_type == ConnectionType::UNIX_SOCKET) {
    switch (connection_target) {
      case ConnectionTarget::SYSTEM:
        connect_to_qb2 = [host = qb2.host.system_socket]() { return connectToUnixSocket(host); };
        break;
      case ConnectionTarget::CORE_PROCESSING:
        connect_to_qb2 = [host = qb2.host.core_processing_socket]() { return connectToUnixSocket(host); };
        break;
      default:
        throw std::runtime_error("Connection to unix socket target is not supported!");
        break;
    }
  } else {
    connect_to_qb2 = [qb2 = qb2]() {
      return hardware::connect_to_device(qb2.host.fqdn, qb2.host.serial_number, qb2.host.application_key);
    };
  }

  /// create channel
  try {
    auto qb2_channel = connect_to_qb2();
    RCLCPP_INFO_STREAM(logger, "Connected to Qb2: " << qb2.hostName() << ".");

    return qb2_channel;
  } catch (const std::runtime_error& exception) {
    RCLCPP_WARN_STREAM(logger,
                       "Failed to connect to Qb2: " << qb2.hostName() << " , due to: " << exception.what() << ".");
    return nullptr;
  }
}

void disconnect(Qb2ChannelPtr& qb2_channel, const Qb2Info& qb2, const rclcpp::Logger& logger) {
  if (qb2_channel != nullptr) {
    qb2_channel = nullptr;
    RCLCPP_INFO_STREAM(logger, "Disconnected from Qb2: " << qb2.hostName() << ".");
  }
}

bool isConnected(const Qb2ChannelPtr& qb2_channel) { return qb2_channel ? true : false; }

bool hasQb2CommunicationFailed(CommunicationState communication_state) {
  bool failure = false;
  switch (communication_state) {
    case CommunicationState::FAIL_READ:
    case CommunicationState::FAIL_NO_CONNECTION:
    case CommunicationState::FAIL_EXCEPTION:
    case CommunicationState::FAIL_AUTHENTICATION:
      failure = true;
      break;
    default:
      break;
  }
  return failure;
}

}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
