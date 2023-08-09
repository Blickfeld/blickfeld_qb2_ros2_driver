#include "qb2_lidar_ros.h"
#include "qb2_ros2_utils.h"

#include <blickfeld/hardware/client.h>

#include <grpc++/create_channel.h>

namespace blickfeld {
namespace ros_interop {

Qb2LidarRos::Qb2LidarRos(rclcpp::Node::SharedPtr node, const std::string& host, const bool& use_socket_connection,
                         const std::string& frame_id, const std::string& point_cloud_topic, bool snapshot_mode)
    : node_(node),
      host_(host),
      use_socket_connection_(use_socket_connection),
      frame_id_(frame_id),
      point_cloud_topic_(point_cloud_topic),
      snapshot_mode_(snapshot_mode) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'host' is set to: " << host_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'frame_id' is set to: " << frame_id_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'point_cloud_topic' is set to: " << point_cloud_topic_);
  /// setup parameters
  if (node_->get_parameter("publish_intensity", point_fields_.intensity) == false) {
    point_fields_.intensity = node_->declare_parameter<bool>("publish_intensity", point_fields_.intensity);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "The 'publish_intensity' is set to: " << (point_fields_.intensity ? "True" : "False"));

  if (node_->get_parameter("publish_point_id", point_fields_.point_id) == false) {
    point_fields_.point_id = node_->declare_parameter<bool>("publish_point_id", point_fields_.point_id);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "The 'publish_point_id' is set to: " << (point_fields_.point_id ? "True" : "False"));

  /// create the ros publisher
  point_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic, 4);
}
Qb2LidarRos::~Qb2LidarRos() { disconnect(); }

const std::string& Qb2LidarRos::getHost() const { return host_; }

bool Qb2LidarRos::readFrame(Qb2Frame& frame) {
  if (snapshot_mode_ == true) {
    return getFrame(frame);
  } else {
    return readFrameFromStream(frame);
  }
}

void Qb2LidarRos::publishFrame(const Qb2Frame& frame, rclcpp::Time timestamp) const {
  auto point_cloud = convertToPointCloudMsg(frame, frame_id_, point_fields_, timestamp);

  point_cloud_publisher_->publish(std::move(point_cloud));
}

bool Qb2LidarRos::readFrameFromStream(Qb2Frame& frame) {
  bool success = false;
  /// (if connected or can connect successfully) and (if streaming or can open a frame stream) read a frame form Qb2
  /// stream
  if ((isConnected() == true || connect()) && (isStreaming() == true || openFrameStream())) {
    Qb2PointCloudStreamResponse response;
    success = point_cloud_stream_->Read(&response);
    if (success == true) {
      /// prepare for 'stealing' the frame using swap.
      response.mutable_frame()->Swap(&frame);
    } else {
      /// HINT: socket connection loss is much more severe than a network hiccup
      if (use_socket_connection_ == true) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get data from socket: " << host_ << "!");
      } else {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to get data from fqdn:" << host_ << "!");
      }
      disconnect();
    }
  }
  return success;
}

bool Qb2LidarRos::getFrame(Qb2Frame& frame) {
  bool success = false;
  /// if connected or can connect successfully get a frame from Qb2
  if (isConnected() == true || connect()) {
    /// HINT: try the GET point cloud API to get a single point cloud from Qb2
    try {
      grpc::ClientContext context;
      Qb2PointCloudGetResponse response;
      auto point_cloud_stub = core_processing::services::PointCloud::NewStub(qb2_channel_);
      auto status = point_cloud_stub->Get(&context, Qb2PointCloudGetRequest(), &response);

      /// success if status is ok
      success = status.ok() ? true : false;
      if (success == true) {
        /// prepare for 'stealing' the frame using swap.
        response.mutable_frame()->Swap(&frame);
        RCLCPP_DEBUG_STREAM(node_->get_logger(), "Got a point cloud from Qb2: " << host_ << ".");
      } else {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to get a point cloud from Qb2: "
                                                    << host_ << " , due to grpc status: " << status.error_code()
                                                    << " - " << status.error_message() << ".");
      }
    } catch (const std::runtime_error& exception) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Failed to get a point cloud from Qb2: " << host_ << " , due to: " << exception.what() << ".");
    }

    /// HINT: GET API failed (maybe due to old firmware) try creating a temp stream and get one frame it
    if (success == false) {
      success = readFrameFromStream(frame);
      closeFrameStream();
    }
  }
  return success;
}

bool Qb2LidarRos::connect() {
  /// create the correct connection method
  std::function<std::shared_ptr<grpc::Channel>()> connect_to_qb2;
  if (use_socket_connection_ == true) {
    connect_to_qb2 = [host = host_]() { return connectToUnixSocket(host); };
  } else {
    connect_to_qb2 = [host = host_]() { return hardware::connect_to_device(host); };
  }

  /// create channel
  try {
    qb2_channel_ = connect_to_qb2();
  } catch (const std::runtime_error& exception) {
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "Failed to connect to Qb2: " << host_ << " , due to: " << exception.what() << ".");
    disconnect();
    return false;
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Connected to Qb2: " << host_ << ".");
  return true;
}

void Qb2LidarRos::disconnect() {
  closeFrameStream();
  qb2_channel_ = nullptr;
}

bool Qb2LidarRos::isConnected() const { return qb2_channel_ ? true : false; }

bool Qb2LidarRos::openFrameStream() {
  /// renew stream context
  stream_context_ = std::make_unique<grpc::ClientContext>();
  try {
    auto point_cloud_stub = core_processing::services::PointCloud::NewStub(qb2_channel_);
    point_cloud_stream_ = point_cloud_stub->Stream(stream_context_.get(), Qb2PointCloudStreamRequest());
  } catch (const std::runtime_error& exception) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to opened a point cloud stream to Qb2: "
                                                << host_ << " , due to: " << exception.what() << "'.");
    closeFrameStream();
    return false;
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Opened a point cloud stream to Qb2: " << host_ << ".");
  return true;
}

void Qb2LidarRos::closeFrameStream() {
  point_cloud_stream_ = nullptr;
  stream_context_->TryCancel();
  stream_context_ = nullptr;
}

bool Qb2LidarRos::isStreaming() const { return point_cloud_stream_ ? true : false; }

}  // namespace ros_interop
}  // namespace blickfeld
