#include "qb2/point_cloud_streamer.h"

#include "utility/qb2_connection_utils.h"
#include "utility/qb2_ros2_utils.h"

#include <string>

namespace blickfeld {
namespace ros_interop {
namespace qb2 {

std::pair<CommunicationState, std::optional<Qb2Frame>> PointCloudStreamer::readFrame() {
  std::pair<CommunicationState, std::optional<Qb2Frame>> read_frame_result = tryReadFrame();
  if (hasQb2CommunicationFailed(read_frame_result.first)) {
    disconnect();
  }
  return read_frame_result;
}

std::pair<CommunicationState, std::optional<Qb2Frame>> PointCloudStreamer::tryReadFrame() {
  if (isConnected(qb2_channel_) == false) {
    qb2_channel_ = connect(qb2_, ConnectionTarget::CORE_PROCESSING, node_->get_logger());
    if (isConnected(qb2_channel_) == false) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Failed to read a point cloud from stream, Qb2: " << qb2_.hostName() << " is disconnected!");
      return std::make_pair(CommunicationState::FAIL_NO_CONNECTION, std::nullopt);
    }
  }

  if (isStreamingPointCloud() == false) {
    CommunicationState state = openPointCloudStream();
    if (isStreamingPointCloud() == false) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to read point cloud, point cloud stream of Qb2: "
                                                  << qb2_.hostName() << " is not available!");
      return std::make_pair(state, std::nullopt);
    }
  }

  try {
    Qb2PointCloudStreamResponse response;
    const bool success = point_cloud_stream_->Read(&response);
    if (success == true) {
      Qb2Frame frame;
      /// prepare for 'stealing' the frame using swap.
      response.mutable_frame()->Swap(&frame);
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Read a point cloud from Qb2 stream: " << qb2_.hostName() << ".");
      return std::make_pair(CommunicationState::SUCCESS_READ, std::move(frame));
    } else {
      /// HINT: The reason for an unsuccessful point cloud stream read could be that a cancel request was sent to the
      /// stream or that something else went wrong. If a cancel event has been sent, the point_cloud_stream_
      /// object is nullptr, so we can't get an status from the point_cloud_stream_ object.
      /// If something else went wrong the point_cloud_stream_ object is still valid and finish should be called
      /// to get the status.

      if (point_cloud_stream_ != nullptr) {
        const grpc::Status status = point_cloud_stream_->Finish();
        RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to read a point cloud from Qb2 stream: "
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
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to read a point cloud from stream of Qb2: "
                                                << qb2_.hostName() << " , due to: " << exception.what() << "!");
    return std::make_pair(CommunicationState::FAIL_EXCEPTION, std::nullopt);
  }
}

CommunicationState PointCloudStreamer::openPointCloudStream() {
  if (isConnected(qb2_channel_) == false) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to open a point cloud stream to Qb2: "
                                                << qb2_.hostName() << " , due to: missing connection!");
    return CommunicationState::FAIL_NO_CONNECTION;
  }

  /// renew stream context
  point_cloud_context_ = std::make_unique<grpc::ClientContext>();
  try {
    auto point_cloud_stub = core_processing::services::PointCloud::NewStub(qb2_channel_);
    point_cloud_stream_ = point_cloud_stub->Stream(point_cloud_context_.get(), Qb2PointCloudStreamRequest());

    RCLCPP_INFO_STREAM(node_->get_logger(), "Opened a point cloud stream to Qb2: " << qb2_.hostName() << ".");
    return CommunicationState::SUCCESS_OPEN_STREAM;

  } catch (const std::runtime_error& exception) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to open a point cloud stream to Qb2: "
                                                << qb2_.hostName() << " , due to: " << exception.what() << "!");

    return CommunicationState::FAIL_EXCEPTION;
  }
}

bool PointCloudStreamer::isStreamingPointCloud() const { return point_cloud_stream_ ? true : false; }

void PointCloudStreamer::disconnect() {
  qb2::disconnect(qb2_channel_, qb2_, node_->get_logger());
  point_cloud_context_ = nullptr;
  point_cloud_stream_ = nullptr;
}

}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
