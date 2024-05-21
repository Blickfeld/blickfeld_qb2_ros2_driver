/**
 * @file
 * @copyright Copyright (C) 2024, Blickfeld GmbH
 */

#pragma once

#include "qb2_driver_status.h"
#include "qb2_ros2_type.h"

#include <rclcpp/rclcpp.hpp>

#include <optional>

namespace blickfeld {
namespace ros_interop {
namespace qb2 {

class ScanPatternWatcher {
 public:
  ScanPatternWatcher(rclcpp::Node::SharedPtr node, const Qb2Info& qb2);

  ~ScanPatternWatcher();

  /**
   * @brief Watches the scan pattern changes on the Qb2
   *
   * @return std::pair<CommunicationState, std::optional<Qb2ScanPattern>> The state of communication and the optional
   * new scan pattern object
   */
  std::pair<CommunicationState, std::optional<Qb2ScanPattern>> watchScanPattern();
  
  /**
   * @brief Try to cancel the running watch stream
   *
   */
  void cancel();


 private:
  /**
   * @brief tries opening a scan patter watch stream and returns a response of the stream.
   * The method doesn't cleanup the internal state after connection issues
   *
   * @return std::pair<CommunicationState, std::optional<Qb2ScanPattern>> The state of communication and the optional
   * new scan pattern object
   */
  std::pair<CommunicationState, std::optional<Qb2ScanPattern>> tryWatchScanPattern();

  /**
   * @brief Attempts to open a scan pattern watch if the qb2_channel is connected
   *
   * @return The state of communication with Qb2, if successful it should return
   * CommunicationState::SUCCESS_OPEN_STREAM
   */
  CommunicationState openScanPatternWatch();
  
    /**
   * @brief disconnects from the server and cleans up all internal connection state
   */
  void disconnect();

  /**
   * @brief Checks if the watch stream is open
   *
   * @return true
   * @return false
   */
  bool isWatchingScanPattern() const;

  rclcpp::Node::SharedPtr node_;

  /// Device info that we want to connect to
  Qb2Info qb2_;

  /// Connection channel to the Qb2 gRPC server
  std::shared_ptr<grpc::Channel> qb2_channel_;

  /// The gRPC client context for Qb2 streams
  std::unique_ptr<grpc::ClientContext> scan_pattern_watch_context_ = nullptr;

  /// Unique pointers for Qb2 streams
  std::unique_ptr<grpc::ClientReader<Qb2ScanPatternWatchResponse>> scan_pattern_watch_ = nullptr;
};
}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
