/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include <blickfeld/core_processing/data/frame.pb.h>
#include <blickfeld/core_processing/services/point_cloud.grpc.pb.h>
#include <blickfeld/system/services/scan_pattern.grpc.pb.h>

#include <grpc++/create_channel.h>
#include <memory>
#include <string>

#include <chrono>

namespace blickfeld {
namespace ros_interop {

using Qb2Frame = core_processing::data::Frame;
using Qb2ClientContextUPtr = std::unique_ptr<grpc::ClientContext>;
using Qb2ChannelPtr = std::shared_ptr<grpc::Channel>;

using Qb2PointCloudStreamRequest = core_processing::services::PointCloudStreamRequest;
using Qb2PointCloudStreamResponse = core_processing::services::PointCloudStreamResponse;
using Qb2PointCloudClientReader = grpc::ClientReader<Qb2PointCloudStreamResponse>;
using Qb2PointCloudClientReaderUPtr = std::unique_ptr<Qb2PointCloudClientReader>;

using Qb2PointCloudGetRequest = core_processing::services::PointCloudGetRequest;
using Qb2PointCloudGetResponse = core_processing::services::PointCloudGetResponse;

using Qb2ScanPatternWatchResponse = system::services::ScanPatternWatchResponse;
using Qb2ScanPatternClientReader = grpc::ClientReader<Qb2ScanPatternWatchResponse>;
using Qb2ScanPatternClientReaderUPtr = std::unique_ptr<Qb2ScanPatternClientReader>;

/// Qb2Info
struct PointCloudInfo {
  /// Name of the ROS-TF frame for publishing the LiDAR data in
  std::string frame_id = {"lidar"};
  /// ROS topic to publish the point cloud from Qb2
  std::string topic = {"~/point_cloud_out"};
  /// Flags for output point fields in addition to (x,y,z)
  bool intensity = true;
  bool point_id = true;
};

struct Snapshot {
  /// Use the Qb2 for live streaming or in snapshot mode
  bool mode = false;
  /// Frame rate for the snapshot
  float frame_rate = 0.f;
};

enum class ConnectionTarget { SYSTEM, CORE_PROCESSING };
enum class ConnectionType { FQDN, UNIX_SOCKET };

struct Host {
  std::string fqdn = {""};
  std::string system_socket = {"/var/run/system.grpc.bf"};
  std::string core_processing_socket = {"/var/run/core_processing.grpc.bf"};
  ConnectionType connection_type = ConnectionType::UNIX_SOCKET;
  std::string application_key = {""};
  std::string serial_number = {""};
};

struct Qb2Info {
  /// host that we want to stream/get point cloud from this could be either fqdn or a unix socket
  Host host;

  PointCloudInfo point_cloud_info;
  Snapshot snapshot;

  std::string hostName() const {
    if (host.connection_type == ConnectionType::UNIX_SOCKET) {
      return host.core_processing_socket;
    } else {
      return host.fqdn;
    }
  }
};

/// Qb2RuntimeStatus
enum class CommunicationState {
  NOT_DEFINED,
  SUCCESS_READ,
  SUCCESS_OPEN_STREAM,
  FAIL_READ,
  FAIL_NO_CONNECTION,
  FAIL_EXCEPTION,
  FAIL_AUTHENTICATION,
  DISCONNECTED
};

struct Qb2RuntimeStatus {
  bool last_frame_success = false;
  double last_frame_duration = 0.;
  std::chrono::system_clock::time_point last_receive_frame_time;
  /// HINT: Qb2 frame id is the incremental number since start of the device/qb2 that will come with the Qb2Frame
  uint64_t last_frame_id = 0;

  uint64_t total_frames_published = 0;
  uint64_t total_frames_dropped = 0;
  uint64_t total_reboots = 0;

  CommunicationState frame_communication_state = CommunicationState::NOT_DEFINED;
  CommunicationState scan_pattern_communication_state = CommunicationState::NOT_DEFINED;
};

/// Qb2ScanPattern
struct Qb2ScanPattern {
  /// angles are in rad
  float horizontal_field_of_view = 1.57f;
  float vertical_field_of_view = 0.87f;
  int scanlines_up = 40;
  int scanlines_down = 40;
  float angle_spacing = 0.00436f;
  float frame_rate = -1.f;
};

}  // namespace ros_interop
}  // namespace blickfeld
