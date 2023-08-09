#include "qb2_ros2_snapshot_driver.h"

#include <google/protobuf/timestamp.pb.h>
#include <rclcpp/exceptions.hpp>

#include <string>

namespace blickfeld {
namespace ros_interop {

Qb2SnapshotDriver::Qb2SnapshotDriver(rclcpp::NodeOptions options)
    : node_(std::make_shared<rclcpp::Node>("blickfeld_qb2_snapshot_driver", options.use_intra_process_comms(true))) {
  /// setup parameters
  use_measurement_timestamp_ = node_->declare_parameter<bool>("use_measurement_timestamp", use_measurement_timestamp_);
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "The 'use_measurement_timestamp' is set to: " << (use_measurement_timestamp_ ? "True" : "False"));
  const double snapshot_frequency = node_->declare_parameter<double>("snapshot_frequency", 0.1);

  if (snapshot_frequency < min_allowed_snapshot_frequency_ || snapshot_frequency > max_allowed_snapshot_frequency_) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "The 'snapshot_frequency' can only be in this range ["
                                                 << min_allowed_snapshot_frequency_ << ", "
                                                 << max_allowed_snapshot_frequency_ << "].");
    rclcpp::shutdown();
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'snapshot_frequency' is set to: " << snapshot_frequency);

  const std::vector<std::string> fqdns =
      node_->declare_parameter<std::vector<std::string>>("fqdns", std::vector<std::string>());
  const std::vector<std::string> fqdn_frame_ids =
      node_->declare_parameter<std::vector<std::string>>("fqdn_frame_ids", std::vector<std::string>());
  const std::vector<std::string> fqdn_point_cloud_topics =
      node_->declare_parameter<std::vector<std::string>>("fqdn_point_cloud_topics", std::vector<std::string>());

  const std::vector<std::string> unix_sockets =
      node_->declare_parameter<std::vector<std::string>>("unix_sockets", std::vector<std::string>());
  const std::vector<std::string> unix_socket_frame_ids =
      node_->declare_parameter<std::vector<std::string>>("unix_socket_frame_ids", std::vector<std::string>());
  const std::vector<std::string> unix_socket_point_cloud_topics =
      node_->declare_parameter<std::vector<std::string>>("unix_socket_point_cloud_topics", std::vector<std::string>());

  if (fqdns.empty() && unix_sockets.empty()) {
    RCLCPP_FATAL(node_->get_logger(), "Neither unix socket nor fqdn were provided!");
    rclcpp::shutdown();
  }

  const std::vector<size_t> fqdn_parameter_sizes{fqdns.size(), fqdn_frame_ids.size(), fqdn_point_cloud_topics.size()};
  if (!std::equal(fqdn_parameter_sizes.begin(), fqdn_parameter_sizes.end(), fqdn_parameter_sizes.begin())) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Sizes of parameters don't match: fqdns: "
                                                 << fqdns.size() << ", fqdn_frame_ids: " << fqdn_frame_ids.size()
                                                 << ", fqdn_point_cloud_topics: " << fqdn_point_cloud_topics.size());
    rclcpp::shutdown();
  }

  const std::vector<size_t> socket_parameter_sizes{unix_sockets.size(), unix_socket_frame_ids.size(),
                                                   unix_socket_point_cloud_topics.size()};
  if (!std::equal(socket_parameter_sizes.begin(), socket_parameter_sizes.end(), socket_parameter_sizes.begin())) {
    RCLCPP_FATAL_STREAM(node_->get_logger(),
                        "Sizes of parameters don't match: unix_sockets: "
                            << unix_sockets.size() << ", unix_socket_frame_ids: " << unix_socket_frame_ids.size()
                            << ", unix_socket_point_cloud_topics: " << unix_socket_point_cloud_topics.size());
    rclcpp::shutdown();
  }

  setupQb2sWithFqdns(fqdns, fqdn_frame_ids, fqdn_point_cloud_topics);
  setupQb2sWithUnixSockets(unix_sockets, unix_socket_frame_ids, unix_socket_point_cloud_topics);

  /// create a timer to trigger the snapshot with a fixed frequency only of the snapshot_frequency is not 0
  if (snapshot_frequency != 0) {
    std::chrono::duration<double> snapshot_duration(1. / snapshot_frequency);
    snapshot_timer_ = node_->create_wall_timer(snapshot_duration, [this](rclcpp::TimerBase&) { this->snapshot(); });
  }

  /// setup a service to manually trigger the snapshot
  trigger_snapshot_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "trigger_snapshot",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) { response->success = this->snapshot(); });
}

Qb2SnapshotDriver::~Qb2SnapshotDriver() {}

void Qb2SnapshotDriver::setupQb2sWithFqdns(const std::vector<std::string>& fqdns,
                                           const std::vector<std::string>& frame_ids,
                                           const std::vector<std::string>& point_cloud_topics) {
  for (size_t qb2_idx = 0; qb2_idx < fqdns.size(); ++qb2_idx) {
    qb2s_.emplace_back(std::make_unique<Qb2LidarRos>(node_, fqdns[qb2_idx], false, frame_ids[qb2_idx],
                                                     point_cloud_topics[qb2_idx], snapshot_mode_));
  }
}

void Qb2SnapshotDriver::setupQb2sWithUnixSockets(const std::vector<std::string>& unix_sockets,
                                                 const std::vector<std::string>& frame_ids,
                                                 const std::vector<std::string>& point_cloud_topics) {
  for (size_t qb2_idx = 0; qb2_idx < unix_sockets.size(); ++qb2_idx) {
    qb2s_.emplace_back(std::make_unique<Qb2LidarRos>(node_, unix_sockets[qb2_idx], true, frame_ids[qb2_idx],
                                                     point_cloud_topics[qb2_idx], snapshot_mode_));
  }
}

bool Qb2SnapshotDriver::snapshot() {
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Snapshot a frame from all Qb2s.");
  /// read all frames
  std::vector<Qb2Frame> frames;
  for (auto& qb2 : qb2s_) {
    Qb2Frame frame;
    auto success = qb2->readFrame(frame);
    if (success == false) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Failed reading the frame of Qb2: " << qb2->getHost() << ", Skipping the entire snapshot.");
      return false;
    }
    frames.emplace_back(std::move(frame));
  }

  /// get one common timestamp to all frames
  rclcpp::Time snapshots_timestamp;
  if (use_measurement_timestamp_ == true) {
    const google::protobuf::uint64 oldest_timestamp =
        std::min_element(frames.begin(), frames.end(), [](const Qb2Frame& a, const Qb2Frame& b) {
          return a.timestamp() < b.timestamp();
        })->timestamp();
    snapshots_timestamp = rclcpp::Time(oldest_timestamp);
  } else {
    snapshots_timestamp = node_->now();
  }

  /// publish the frames on their respective topic
  int frame_index = 0;
  for (auto& qb2 : qb2s_) {
    qb2->publishFrame(frames[frame_index], snapshots_timestamp);
    frame_index++;
  }
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Published " << qb2s_.size() << " point clouds.");
  return true;
}

}  // namespace ros_interop
}  // namespace blickfeld

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blickfeld::ros_interop::Qb2SnapshotDriver)
