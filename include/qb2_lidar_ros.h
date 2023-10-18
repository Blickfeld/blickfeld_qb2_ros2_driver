

/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include "qb2_ros2_type.h"

#include <blickfeld/core_processing/services/point_cloud.grpc.pb.h>

#include <grpc++/client_context.h>
#include <grpc++/create_channel.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace blickfeld {
namespace ros_interop {

/**
 * @class blickfeld::ros_interop::Qb2LidarRos
 *
 * @brief Class wrapping the grpc communication with Qb2, providing functions to read and publish frames from Qb2
 */
class Qb2LidarRos {
  using Qb2PointCloudStreamRequest = core_processing::services::PointCloudStreamRequest;
  using Qb2PointCloudStreamResponse = core_processing::services::PointCloudStreamResponse;
  using Qb2PointCloudGetRequest = core_processing::services::PointCloudGetRequest;
  using Qb2PointCloudGetResponse = core_processing::services::PointCloudGetResponse;

 public:
  /**
   * @brief Constructor
   *
   * @param[in] node the driver node
   * @param[in] host the host (Qb2) to connect to and get data from, this is either fqdn or unix socket
   * @param[in] use_socket_connection the flag to identify if the host is fqdn or unix socket
   * @param[in] frame_id the ROS TF frame for publishing the point cloud
   * @param[in] point_cloud_topic the ROS topic for publishing the point cloud
   * @param[in] snapshot_mode the flag to use the Qb2 with snapshot or live stream mode
   * @param[in, out]  diagnostic_updater the diagnostic updater for registering the diagnostic update function
   */
  Qb2LidarRos(rclcpp::Node::SharedPtr node, const std::string& host, const bool& use_socket_connection,
              const std::string& frame_id, const std::string& point_cloud_topic, bool snapshot_mode,
              diagnostic_updater::Updater& diagnostic_updater);

  /**
   * @brief Destructor
   */
  ~Qb2LidarRos();

  /**
   * @brief Get the host/name of Qb2
   *
   * @return the host set for the connected Qb2
   */
  const std::string& getHost() const;

  /**
   * @brief Read one frame from Qb2
   *
   * @param [in, out] frame to be filled from Qb2
   * @return returns true if getting one frame from Qb2 was successful
   */
  bool readFrame(Qb2Frame& frame);

  /**
   * @brief publish the frame as a PointCloud2 msg
   * @param[in] frame the data from Qb2
   * @param[in] timestamp the timestamp to set in point cloud header
   */
  void publishFrame(const Qb2Frame& frame, rclcpp::Time timestamp);

 private:
  /**
   * @brief Establishes a connection to Qb2 if not connected already and opens a point cloud stream if stream is not
   * established yet and reads a frame from Qb2 Stream.
   * If connection fails it returns a frame that does not contain a value.
   * If establishing a stream fails it returns a frame that does not contain a value.
   * If reading from a stream fails it logs the event and stops the stream form Qb2 then returns a frame that does not
   * contain a value
   * @param [in, out] frame to be filled from Qb2
   * @return returns true if reading from a stream was successful
   */
  bool readFrameFromStream(Qb2Frame& frame);

  /**
   * @brief Established a connection to Qb2 if not connected already and get a single frame from Qb2 with GET API
   * If not successful with using GET API use readFrameFromStream but close the stream after getting the frame
   * @param [in, out] frame to be filled from Qb2
   * @return returns true if getting one frame from Qb2 was successful
   */
  bool getFrame(Qb2Frame& frame);

  /**
   * @brief Creates a new grpc channel and attempts to connect and open a point cloud stream
   * If the connection fails the disconnect function is called.
   * @return True on success
   */
  bool connect();

  /**
   * @brief Resets the point cloud stream and therfore disconnects from the device
   */
  void disconnect();

  /**
   * @brief Checks the state of connection to Qb2
   *
   * @return true if connection channel is established
   */
  bool isConnected() const;

  /**
   * @brief Opens a stream to Qb2 for reading frame
   *
   * @return True on success
   */
  bool openFrameStream();

  /**
   * @brief Closes the stream to Qb2 for reading frame
   *
   * @return True on success
   */
  void closeFrameStream();

  /**
   * @brief Checks if the point cloud stream is established
   *
   * @return true if the point cloud stream is open otherwise false
   */
  bool isStreaming() const;

  /**
   * @brief Updates status for the ros diagnostic updater
   *
   * @param[in, out] status the status to update in diagnostic updater
   */
  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status);

  rclcpp::Node::SharedPtr node_;

  /// host that we want to stream/get point cloud from this could be either fqdn or a unix socket
  std::string host_ = {""};
  bool use_socket_connection_ = true;

  /// Flags for output point fields in addition to (x,y,z)
  PointFields point_fields_;
  /// Name of the ROS-TF frame for publishing the LiDAR data in
  std::string frame_id_ = {"lidar"};
  /// ROS topic to publish the point cloud from Qb2
  std::string point_cloud_topic_ = {"~/point_cloud_out"};

  /// Use the Qb2 for live streaming or in snapshot mode
  bool snapshot_mode_ = false;

  /// ROS publisher for point cloud
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> point_cloud_publisher_;

  /// Connection channel to the Qb2 gRPC server
  std::shared_ptr<grpc::Channel> qb2_channel_;

  /// The gRPC client context for stream
  std::unique_ptr<grpc::ClientContext> stream_context_ = nullptr;

  /// Unique pointers for the point cloud stream of Qb2
  std::unique_ptr<grpc::ClientReader<Qb2PointCloudStreamResponse>> point_cloud_stream_ = nullptr;

  /// Timeout of 60 second for qb2 api
  static constexpr uint64_t qb2_api_timeout_ms_ = 60000;

  struct DriverStatus {
    uint64_t failed_connection_attempts_ = 0;
    uint64_t connection_attempts_since_last_connection_ = 0;

    uint64_t failed_opening_stream_attempts_ = 0;
    uint64_t opening_stream_attempts_since_last_opened_stream_ = 0;

    bool last_frame_success_ = false;
    double last_frame_duration_ = 0.;

    uint64_t total_frames_published_ = 0;

    /// HINT: Qb2 frame id is the incremental number since start of the device/qb2 that will come with the Qb2Frame
    uint64_t last_frame_id_ = 0;
    uint64_t total_frames_dropped_ = 0;

    uint64_t total_reboots_ = 0;

    void onConnect() {
      /// HINT: since connection was successful we reset the counter
      connection_attempts_since_last_connection_ = 0;
    }
    void failedToConnect() {
      failed_connection_attempts_++;
      connection_attempts_since_last_connection_++;
    }

    void onOpeningStream() {
      /// HINT: since stream was opened successfully we reset the counter
      opening_stream_attempts_since_last_opened_stream_ = 0;
    }
    void failedToOpenStream() {
      failed_opening_stream_attempts_++;
      opening_stream_attempts_since_last_opened_stream_++;
    }

    void onPublishingFrame() { total_frames_published_++; }

    /**
     * @brief Updates the driver status struc based on the input after each frame this should be called
     *
     * @param[in] request_time the time of the sending the request
     * @param[in] receive_time the time of the receiving the response
     * @param[in] success the status of the request
     * @param[in] frame_id the counter of the frame received from Qb2
     * @param[in] snapshot_mode the mode that the driver is running
     * @return true if reboot was detected otherwise false
     */
    bool updateDriverStatus(const std::chrono::system_clock::time_point& request_time,
                            const std::chrono::system_clock::time_point& receive_time, bool success, uint64_t frame_id,
                            bool snapshot_mode) {
      auto reboot = updateRebootCounter(frame_id);
      updateLastFrameStatus(request_time, receive_time, success, frame_id);
      updateFrameDrop(snapshot_mode, frame_id);
      return reboot;
    }

   private:
    /**
     * @brief Calculates the last frame request duration and set the status
     *
     * @param[in] request_time the time of the sending the request
     * @param[in] receive_time the time of the receiving the response
     * @param[in] success the status of the request
     * @param[in] frame_id the counter of the frame of Qb2
     */
    void updateLastFrameStatus(const std::chrono::system_clock::time_point& request_time,
                               const std::chrono::system_clock::time_point& receive_time, bool success,
                               uint64_t frame_id) {
      last_frame_duration_ = std::chrono::duration<double>(receive_time - request_time).count();
      last_frame_success_ = success;
      if (last_frame_id_ == 0) {
        last_frame_id_ = frame_id;
      }
    }

    /**
     * @brief Calculate the number of detected reboots of the Qb2
     *
     * @param[in] frame_id The counter of the frame received from Qb2
     * @return true if the reboot attempt was detected otherwise false
     */
    bool updateRebootCounter(uint64_t frame_id) {
      const bool qb2_rebooted = frame_id < last_frame_id_;
      if (qb2_rebooted == true) {
        total_reboots_++;
      }
      return qb2_rebooted;
    }

    /**
     * @brief Calculate and set the number of dropped frames
     *
     * @param[in] snapshot_mode the mode that the driver is running
     * @param[in] frame_id the counter of the frame received from Qb2
     */
    void updateFrameDrop(bool snapshot_mode, uint64_t frame_id) {
      if (last_frame_success_ == true) {
        /// HINT: in stream mode we can calculate the dropped frames after receiving the first frame
        if (frame_id > last_frame_id_ && snapshot_mode == false) {
          /// HINT: if the difference between frame ids is 1 then we have not dropped any frames
          total_frames_dropped_ += frame_id - last_frame_id_ - 1;
        }
        last_frame_id_ = frame_id;
      } else {
        if (snapshot_mode == true) {
          total_frames_dropped_++;
        }
      }
    }
  };
  DriverStatus driver_status_;
};

}  // namespace ros_interop
}  // namespace blickfeld
