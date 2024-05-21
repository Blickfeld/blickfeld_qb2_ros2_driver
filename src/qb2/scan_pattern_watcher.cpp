#include "qb2/scan_pattern_watcher.h"

#include "utility/qb2_connection_utils.h"
#include "utility/qb2_ros2_utils.h"

namespace blickfeld {
namespace ros_interop {
namespace qb2 {
ScanPatternWatcher::ScanPatternWatcher(rclcpp::Node::SharedPtr node, const Qb2Info& qb2) : node_(node), qb2_(qb2) {}

ScanPatternWatcher::~ScanPatternWatcher() { cancel(); }

std::pair<CommunicationState, std::optional<Qb2ScanPattern>> ScanPatternWatcher::watchScanPattern() {
  std::pair<CommunicationState, std::optional<Qb2ScanPattern>> watch_scan_pattern_result = tryWatchScanPattern();
  if (hasQb2CommunicationFailed(watch_scan_pattern_result.first)) {
    disconnect();
  }
  return watch_scan_pattern_result;
}

std::pair<CommunicationState, std::optional<Qb2ScanPattern>> ScanPatternWatcher::tryWatchScanPattern() {
  if (isConnected(qb2_channel_) == false) {
    qb2_channel_ = connect(qb2_, ConnectionTarget::SYSTEM, node_->get_logger());
    if (isConnected(qb2_channel_) == false) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Failed to watch scan pattern from stream, Qb2: " << qb2_.hostName() << " is disconnected!");
      return std::make_pair(CommunicationState::FAIL_NO_CONNECTION, std::nullopt);
    }
  }

  if (isWatchingScanPattern() == false) {
    CommunicationState state = openScanPatternWatch();
    if (isWatchingScanPattern() == false) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to watch scan pattern, scan pattern stream of Qb2: "
                                                  << qb2_.hostName() << " is not available!");
      return std::make_pair(state, std::nullopt);
    }
  }

  try {
    Qb2ScanPatternWatchResponse response;
    const bool success = scan_pattern_watch_->Read(&response);

    if (success == true) {
      Qb2ScanPattern scan_pattern;
      /// Update the scan pattern related info
      scan_pattern.horizontal_field_of_view = response.scan_pattern().horizontal().field_of_view();
      scan_pattern.vertical_field_of_view = response.scan_pattern().vertical().field_of_view();
      scan_pattern.scanlines_up = response.scan_pattern().vertical().scanlines_up();
      scan_pattern.scanlines_down = response.scan_pattern().vertical().scanlines_down();
      scan_pattern.angle_spacing = response.scan_pattern().pulse().angle_spacing();
      scan_pattern.frame_rate = response.scan_pattern().frame_rate().maximum();

      RCLCPP_INFO_STREAM(node_->get_logger(), "Qb2 scan pattern updated: " << qb2_.hostName() << ".");
      return std::make_pair(CommunicationState::SUCCESS_READ, scan_pattern);
    } else {
      /// HINT: The reason for an unsuccessful scan pattern watch read could be that a cancel request was sent to the
      /// stream or that something else went wrong. If a cancel event has been sent, the scan_pattern_watch_
      /// object is nullptr, so we can't get an status from the scan_pattern_watch_ object.
      /// If something else went wrong the scan_pattern_watch_ object is still valid and finish should be called
      /// to get the status.
      if (scan_pattern_watch_ != nullptr) {
        grpc::Status status = scan_pattern_watch_->Finish();
        RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to watch scan pattern from Qb2 : "
                                                    << qb2_.hostName() << " , due to grpc status: "
                                                    << status.error_code() << " - " << status.error_message() << "!");

        if (didAuthenticationFail(status) == true) {
          return std::make_pair(CommunicationState::FAIL_AUTHENTICATION, std::nullopt);
        } else {
          return std::make_pair(CommunicationState::FAIL_READ, std::nullopt);
        }
      } else {
        return std::make_pair(CommunicationState::FAIL_READ, std::nullopt);
      }
    }
  } catch (const std::runtime_error& exception) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to watch scan pattern from Qb2: "
                                                << qb2_.hostName() << " , due to: " << exception.what() << "!");
    return std::make_pair(CommunicationState::FAIL_EXCEPTION, std::nullopt);
  }
}

CommunicationState ScanPatternWatcher::openScanPatternWatch() {
  if (isConnected(qb2_channel_) == false) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to open a scan pattern watch stream to Qb2: "
                                                << qb2_.hostName() << " , due to: missing connection!");
    return CommunicationState::FAIL_NO_CONNECTION;
  }

  /// renew stream context
  scan_pattern_watch_context_ = std::make_unique<grpc::ClientContext>();
  try {
    auto scan_pattern_stub = system::services::ScanPattern::NewStub(qb2_channel_);
    scan_pattern_watch_ = scan_pattern_stub->Watch(scan_pattern_watch_context_.get(), google::protobuf::Empty());

    RCLCPP_INFO_STREAM(node_->get_logger(), "Opened a scan pattern watch stream to Qb2: " << qb2_.hostName() << ".");
    return CommunicationState::SUCCESS_OPEN_STREAM;

  } catch (const std::runtime_error& exception) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to open a scan pattern watch stream to Qb2: "
                                                << qb2_.hostName() << " , due to: " << exception.what() << "!");

    return CommunicationState::FAIL_EXCEPTION;
  }
}

bool ScanPatternWatcher::isWatchingScanPattern() const { return scan_pattern_watch_ ? true : false; }

void ScanPatternWatcher::cancel() {
  if (scan_pattern_watch_context_ != nullptr) {
    scan_pattern_watch_context_->TryCancel();
  }
}

void ScanPatternWatcher::disconnect() {
  qb2::disconnect(qb2_channel_, qb2_, node_->get_logger());
  scan_pattern_watch_context_ = nullptr;
  scan_pattern_watch_ = nullptr;
}

}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
