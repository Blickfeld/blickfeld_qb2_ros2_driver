#include "qb2_ros2_snapshot_driver.h"

#include <google/protobuf/timestamp.pb.h>
#include <rclcpp/exceptions.hpp>

#include <string>

namespace blickfeld {
namespace ros_interop {

Qb2SnapshotDriver::Qb2SnapshotDriver(rclcpp::NodeOptions options)
    : node_(std::make_shared<rclcpp::Node>("blickfeld_qb2_snapshot_driver", options.use_intra_process_comms(true))),
      diagnostic_updater_(node_) {
  /// setup parameters
  use_measurement_timestamp_ = node_->declare_parameter<bool>("use_measurement_timestamp", use_measurement_timestamp_);
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "The 'use_measurement_timestamp' is set to: " << (use_measurement_timestamp_ ? "True" : "False"));

  bool intensity = node_->declare_parameter<bool>("publish_intensity", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'publish_intensity' is set to: " << (intensity ? "True" : "False"));

  bool point_id = node_->declare_parameter<bool>("publish_point_id", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'publish_point_id' is set to: " << (point_id ? "True" : "False"));

  max_retries_ = node_->declare_parameter<uint8_t>("max_retries", max_retries_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'max_retries' is set to: " << max_retries_);

  double snapshot_frame_rate = node_->declare_parameter<double>("snapshot_frame_rate", 0.1);
  if (snapshot_frame_rate < min_allowed_snapshot_frame_rate_ ||
      snapshot_frame_rate > max_allowed_snapshot_frame_rate_) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "The 'snapshot_frame_rate' can only be in this range ["
                                                 << min_allowed_snapshot_frame_rate_ << ", "
                                                 << max_allowed_snapshot_frame_rate_ << "].");
    rclcpp::shutdown();
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'snapshot_frame_rate' is set to: " << snapshot_frame_rate);

  /// figure out the hosts and method of connection
  const std::vector<std::string> fqdns =
      node_->declare_parameter<std::vector<std::string>>("fqdns", std::vector<std::string>());
  const std::vector<std::string> fqdn_serial_numbers =
      node_->declare_parameter<std::vector<std::string>>("fqdn_serial_numbers", std::vector<std::string>());
  const std::vector<std::string> fqdn_application_keys =
      node_->declare_parameter<std::vector<std::string>>("fqdn_application_keys", std::vector<std::string>());
  const std::vector<std::string> fqdn_frame_ids =
      node_->declare_parameter<std::vector<std::string>>("fqdn_frame_ids", std::vector<std::string>());
  const std::vector<std::string> fqdn_point_cloud_topics =
      node_->declare_parameter<std::vector<std::string>>("fqdn_point_cloud_topics", std::vector<std::string>());

  const std::vector<std::string> system_unix_sockets =
      node_->declare_parameter<std::vector<std::string>>("system_unix_sockets", std::vector<std::string>());
  const std::vector<std::string> core_processing_unix_sockets =
      node_->declare_parameter<std::vector<std::string>>("core_processing_unix_sockets", std::vector<std::string>());
  const std::vector<std::string> unix_socket_frame_ids =
      node_->declare_parameter<std::vector<std::string>>("unix_socket_frame_ids", std::vector<std::string>());
  const std::vector<std::string> unix_socket_point_cloud_topics =
      node_->declare_parameter<std::vector<std::string>>("unix_socket_point_cloud_topics", std::vector<std::string>());

  if ((system_unix_sockets.empty() || core_processing_unix_sockets.empty()) && fqdns.empty()) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Neither unix socket nor fqdn were provided!");
    rclcpp::shutdown();
  }

  const std::vector<size_t> fqdn_parameter_sizes{fqdns.size(), fqdn_serial_numbers.size(), fqdn_application_keys.size(),
                                                 fqdn_frame_ids.size(), fqdn_point_cloud_topics.size()};
  if (!std::equal(fqdn_parameter_sizes.begin(), fqdn_parameter_sizes.end(), fqdn_parameter_sizes.begin())) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Sizes of parameters don't match: fqdns: "
                                                 << fqdns.size() << ", fqdn_frame_ids: " << fqdn_frame_ids.size()
                                                 << ", fqdn_point_cloud_topics: " << fqdn_point_cloud_topics.size());
    rclcpp::shutdown();
  }

  const std::vector<size_t> socket_parameter_sizes{system_unix_sockets.size(), core_processing_unix_sockets.size(),
                                                   unix_socket_frame_ids.size(), unix_socket_point_cloud_topics.size()};
  if (!std::equal(socket_parameter_sizes.begin(), socket_parameter_sizes.end(), socket_parameter_sizes.begin())) {
    RCLCPP_FATAL_STREAM(node_->get_logger(),
                        "Sizes of parameters don't match: system_unix_sockets: "
                            << system_unix_sockets.size()
                            << ", core_processing_unix_sockets: " << core_processing_unix_sockets.size()
                            << ", unix_socket_frame_ids: " << unix_socket_frame_ids.size()
                            << ", unix_socket_point_cloud_topics: " << unix_socket_point_cloud_topics.size());
    rclcpp::shutdown();
  }

  /// set the name of the driver as the hardwareID of the updater
  diagnostic_updater_.setHardwareID("Blickfeld Qb2 Snapshot Driver");
  diagnostic_updater_.add(heartbeat_);
  stamp_status_ = std::make_shared<diagnostic_updater::TimeStampStatus>(diagnostic_updater::TimeStampStatusParam(),
                                                                        "Read frame time");
  diagnostic_updater_.add(*stamp_status_);

  setupQb2Fqdns(fqdns, fqdn_serial_numbers, fqdn_application_keys, fqdn_frame_ids, fqdn_point_cloud_topics,
                snapshot_frame_rate, intensity, point_id);
  setupQb2UnixSockets(system_unix_sockets, core_processing_unix_sockets, unix_socket_frame_ids,
                      unix_socket_point_cloud_topics, snapshot_frame_rate, intensity, point_id);

  /// create a timer to trigger the snapshot with a fixed frame_rate only if the snapshot_frame_rate is not 0
  if (snapshot_frame_rate != 0) {
    std::chrono::duration<double> snapshot_duration(1. / snapshot_frame_rate);
    snapshot_timer_ =
        node_->create_wall_timer(snapshot_duration, [this](rclcpp::TimerBase&) { this->snapshotTriggerCallback(); });
  }

  /// setup a service to manually trigger the snapshot
  trigger_snapshot_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "trigger_snapshot", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = this->snapshotTriggerCallback();
      });

  /// HINT: In order to get an immediate snapshot we call the function once
  snapshotTriggerCallback();
}

Qb2SnapshotDriver::~Qb2SnapshotDriver() {
  for (auto& qb2 : qb2s_) {
    qb2->shutdownThreads();
  }
  snapshot_thread_.stop();
  snapshot_thread_.join();
}

void Qb2SnapshotDriver::setupQb2Fqdns(const std::vector<std::string>& fqdns,
                                      const std::vector<std::string>& fqdn_serial_numbers,
                                      const std::vector<std::string>& fqdn_application_keys,
                                      const std::vector<std::string>& frame_ids,
                                      const std::vector<std::string>& point_cloud_topics, float snapshot_frame_rate,
                                      bool intensity, bool point_id) {
  for (size_t qb2_idx = 0; qb2_idx < fqdns.size(); ++qb2_idx) {
    Host host;
    host.connection_type = ConnectionType::FQDN;
    host.fqdn = fqdns[qb2_idx];
    host.serial_number = fqdn_serial_numbers[qb2_idx];
    host.application_key = fqdn_application_keys[qb2_idx];
    Qb2Info qb2_info{host, PointCloudInfo{frame_ids[qb2_idx], point_cloud_topics[qb2_idx], intensity, point_id},
                     Snapshot{snapshot_mode_, snapshot_frame_rate}};

    auto point_cloud_reader = std::make_unique<qb2::PointCloudGetter>(node_, qb2_info);
    qb2s_.emplace_back(
        std::make_unique<Qb2LidarRos>(node_, qb2_info, diagnostic_updater_, std::move(point_cloud_reader)));
  }
}

void Qb2SnapshotDriver::setupQb2UnixSockets(const std::vector<std::string>& system_unix_sockets,
                                            const std::vector<std::string>& core_processing_unix_sockets,
                                            const std::vector<std::string>& frame_ids,
                                            const std::vector<std::string>& point_cloud_topics,
                                            float snapshot_frame_rate, bool intensity, bool point_id) {
  for (size_t qb2_idx = 0; qb2_idx < system_unix_sockets.size(); ++qb2_idx) {
    Host host;
    host.connection_type = ConnectionType::UNIX_SOCKET;
    host.system_socket = system_unix_sockets[qb2_idx];
    host.core_processing_socket = core_processing_unix_sockets[qb2_idx];
    Qb2Info qb2_info{host, PointCloudInfo{frame_ids[qb2_idx], point_cloud_topics[qb2_idx], intensity, point_id},
                     Snapshot{snapshot_mode_, snapshot_frame_rate}};

    auto point_cloud_reader = std::make_unique<qb2::PointCloudGetter>(node_, qb2_info);
    qb2s_.emplace_back(
        std::make_unique<Qb2LidarRos>(node_, qb2_info, diagnostic_updater_, std::move(point_cloud_reader)));
  }
}

bool Qb2SnapshotDriver::snapshotTriggerCallback() {
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Try to get a snapshot if one is not running");
  if (snapshot_is_running_ == false) {
    boost::asio::post(snapshot_thread_, [&]() { this->snapshot(); });
    return true;
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to trigger a snapshot. The last snapshot is still running!");
    return false;
  }
}

void Qb2SnapshotDriver::snapshot() {
  snapshot_is_running_ = true;
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Snapshot a frame from all Qb2s.");

  /// read all frames
  rclcpp::Time read_frame_ts = node_->now();
  std::vector<Qb2Frame> frames;
  for (auto& qb2 : qb2s_) {
    std::optional<Qb2Frame> frame;
    uint8_t num_tries = 0;
    do {
      frame = qb2->readFrame();
      num_tries++;
    } while (frame.has_value() == false && num_tries < max_retries_);

    if (frame.has_value() == false) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed reading the frame of Qb2: " << qb2->getQb2Info().hostName()
                                                                                  << ", Skipping the entire snapshot!");
      snapshot_is_running_ = false;
      return;
    }
    frames.emplace_back(std::move(frame.value()));
  }
  stamp_status_->tick(read_frame_ts);

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
  snapshot_is_running_ = false;
}

}  // namespace ros_interop
}  // namespace blickfeld

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blickfeld::ros_interop::Qb2SnapshotDriver)
