#include "qb2/point_cloud_getter.h"

#include "utility/qb2_connection_utils.h"
#include "utility/qb2_ros2_utils.h"

namespace blickfeld {
namespace ros_interop {
namespace qb2 {

std::pair<CommunicationState, std::optional<Qb2Frame>> PointCloudGetter::readFrame() {
  if (isConnected(qb2_channel_) == false) {
    qb2_channel_ = connect(qb2_, ConnectionTarget::CORE_PROCESSING, node_->get_logger());
    if (isConnected(qb2_channel_) == false) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Failed to get a point cloud from, Qb2: " << qb2_.hostName() << " is disconnected!");
      return std::make_pair(CommunicationState::FAIL_NO_CONNECTION, std::nullopt);
    }
  }

  try {
    /// renew get context
    point_cloud_context_ = std::make_unique<grpc::ClientContext>();
    /// Timeout of 60 second for Qb2 Get API
    static constexpr uint64_t qb2_api_timeout_in_seconds = 60;
    auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(qb2_api_timeout_in_seconds);
    point_cloud_context_->set_deadline(deadline);

    Qb2PointCloudGetResponse response;
    auto point_cloud_stub = core_processing::services::PointCloud::NewStub(qb2_channel_);
    grpc::Status status = point_cloud_stub->Get(point_cloud_context_.get(), Qb2PointCloudGetRequest(), &response);

    if (status.ok() == true) {
      Qb2Frame frame;
      /// prepare for 'stealing' the frame using swap.
      response.mutable_frame()->Swap(&frame);
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Got a point cloud from Qb2: " << qb2_.hostName() << ".");
      return std::make_pair(CommunicationState::SUCCESS_READ, frame);

    } else {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to get a point cloud from Qb2: "
                                                  << qb2_.hostName() << " , due to grpc status: " << status.error_code()
                                                  << " - " << status.error_message() << "!");
      if (didAuthenticationFail(status) == true) {
        return std::make_pair(CommunicationState::FAIL_AUTHENTICATION, std::nullopt);
      } else {
        return std::make_pair(CommunicationState::FAIL_READ, std::nullopt);
      }
    }
  } catch (const std::runtime_error& exception) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to get a point cloud from Qb2: " << qb2_.hostName() << " , due to: "
                                                                                     << exception.what() << "!");

    return std::make_pair(CommunicationState::FAIL_EXCEPTION, std::nullopt);
  }
}

}  // namespace qb2
}  // namespace ros_interop
}  // namespace blickfeld
