#include "qb2_lidar_ros.h"
#include "qb2_ros2_utils.h"

#include <blickfeld/hardware/client.h>

#include <grpc++/create_channel.h>

namespace blickfeld {
namespace ros_interop {

Qb2LidarRos::Qb2LidarRos(rclcpp::Node::SharedPtr node, const std::string& host, const bool& use_socket_connection,
                         const std::string& frame_id, const std::string& point_cloud_topic, bool snapshot_mode,
                         diagnostic_updater::Updater& diagnostic_updater)
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
  point_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic, 1);

  diagnostic_updater.add("Diagnostic message", this, &Qb2LidarRos::updateDiagnostic);
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

void Qb2LidarRos::publishFrame(const Qb2Frame& frame, rclcpp::Time timestamp) {
  auto point_cloud = convertToPointCloudMsg(frame, frame_id_, point_fields_, timestamp);
  point_cloud_publisher_->publish(std::move(point_cloud));
  driver_status_.onPublishingFrame();
}

bool Qb2LidarRos::readFrameFromStream(Qb2Frame& frame) {
  bool success = false;
  std::chrono::system_clock::time_point request_read_api_timestamp;
  std::chrono::system_clock::time_point received_read_api_timestamp;
  /// (if connected or can connect successfully) and (if streaming or can open a frame stream) read a frame form Qb2
  /// stream
  if ((isConnected() == true || connect()) && (isStreaming() == true || openFrameStream())) {
    Qb2PointCloudStreamResponse response;
    request_read_api_timestamp = std::chrono::high_resolution_clock::now();
    success = point_cloud_stream_->Read(&response);
    received_read_api_timestamp = std::chrono::high_resolution_clock::now();

    if (success == true) {
      /// prepare for 'stealing' the frame using swap.
      response.mutable_frame()->Swap(&frame);
    } else {
      /// HINT: socket connection loss is much more severe than a network hiccup
      if (use_socket_connection_ == true) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get data from socket: " << host_ << "!");
      } else {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to get data from fqdn: " << host_ << "!");
      }
      disconnect();
    }
  }

  /// HINT: If no swap occurred frame is simply an empty initialized frame, meaning id returns 0
  bool reboot_detected = driver_status_.updateDriverStatus(request_read_api_timestamp, received_read_api_timestamp,
                                                           success, frame.id(), snapshot_mode_);
  if (reboot_detected == true) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Qb2 with fqdn: " << host_ << " has been rebooted!!!");
  }

  return success;
}

bool Qb2LidarRos::getFrame(Qb2Frame& frame) {
  bool success = false;
  std::chrono::system_clock::time_point request_get_api_timestamp;
  std::chrono::system_clock::time_point received_get_api_timestamp;
  /// if connected or can connect successfully get a frame from Qb2
  if (isConnected() == true || connect()) {
    /// HINT: try the GET point cloud API to get a single point cloud from Qb2
    try {
      grpc::ClientContext context;
      auto deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(qb2_api_timeout_ms_);
      context.set_deadline(deadline);
      Qb2PointCloudGetResponse response;
      auto point_cloud_stub = core_processing::services::PointCloud::NewStub(qb2_channel_);

      request_get_api_timestamp = std::chrono::high_resolution_clock::now();
      auto status = point_cloud_stub->Get(&context, Qb2PointCloudGetRequest(), &response);
      received_get_api_timestamp = std::chrono::high_resolution_clock::now();
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
        disconnect();
      }
    } catch (const std::runtime_error& exception) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Failed to get a point cloud from Qb2: " << host_ << " , due to: " << exception.what() << ".");
      /// HINT: GET API failed (maybe due to old firmware) try creating a temp stream and get one frame it
      success = readFrameFromStream(frame);
      closeFrameStream();
    }
  }

  bool reboot_detected = driver_status_.updateDriverStatus(request_get_api_timestamp, received_get_api_timestamp,
                                                           success, frame.id(), snapshot_mode_);
  if (reboot_detected == true) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Qb2 with fqdn: " << host_ << " has been rebooted!!!");
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
  /// TODO: this does not always have a deadline, if the device is not available it will throw an exception immediately
  /// failed (insecure pre-handshake serial number fetch): Connect Failed.
  try {
    qb2_channel_ = connect_to_qb2();
  } catch (const std::runtime_error& exception) {
    RCLCPP_WARN_STREAM(node_->get_logger(),
                       "Failed to connect to Qb2: " << host_ << " , due to: " << exception.what() << ".");
    disconnect();
    driver_status_.failedToConnect();
    return false;
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Connected to Qb2: " << host_ << ".");

  driver_status_.onConnect();
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
    driver_status_.failedToOpenStream();
    return false;
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Opened a point cloud stream to Qb2: " << host_ << ".");

  driver_status_.onOpeningStream();
  return true;
}

void Qb2LidarRos::closeFrameStream() {
  point_cloud_stream_ = nullptr;
  if (stream_context_ != nullptr) {
    stream_context_->TryCancel();
  }
  stream_context_ = nullptr;
}

bool Qb2LidarRos::isStreaming() const { return point_cloud_stream_ ? true : false; }

void Qb2LidarRos::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
  if (isConnected() == true) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Device is connected");
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Device is disconnected");
  }
  status.add("device_fqdn", host_);
  status.add("connection_status", isConnected());
  status.add("failed_connection_attempts", driver_status_.failed_connection_attempts_);
  status.add("connection_attempts_since_last_connection", driver_status_.connection_attempts_since_last_connection_);

  status.add("stream_status", isStreaming());
  status.add("failed_opening_stream_attempts", driver_status_.failed_opening_stream_attempts_);
  status.add("opening_stream_attempts_since_last_opened_stream",
             driver_status_.opening_stream_attempts_since_last_opened_stream_);

  status.add("last_frame_success", driver_status_.last_frame_success_);
  status.add("last_frame_duration", driver_status_.last_frame_duration_);

  status.add("total_frames_published", driver_status_.total_frames_published_);

  status.add("total_frames_dropped", driver_status_.total_frames_dropped_);

  status.add("total_reboots", driver_status_.total_reboots_);
}

}  // namespace ros_interop
}  // namespace blickfeld
