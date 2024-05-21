/**
 * @file
 * @copyright Copyright (C) 2023, Blickfeld GmbH
 */

#pragma once

#include "qb2_ros2_type.h"
#include "utility/qb2_connection_utils.h"

#include <diagnostic_updater/update_functions.hpp>

#include <optional>

namespace blickfeld {
namespace ros_interop {

class DriverStatus {
 public:
  DriverStatus(const Qb2Info& qb2) : qb2_(qb2) {}

  /**
   * @brief Update the number of published frames
   *
   */
  void onPublishingFrame() { runtime_status_.total_frames_published++; }

  /**
   * @brief Updates the driver runtime status struc based on the input after each frame this should be called
   *
   * @param[in] frame_id the counter of the frame received from Qb2
   * @param[in] frame_communication_state the status of the communication
   * @param[in] request_time the time of the sending the request
   * @param[in] receive_time the time of the receiving the response
   */
  void updateRuntimeStatus(uint64_t frame_id, CommunicationState frame_communication_state,
                           const std::chrono::system_clock::time_point& request_time,
                           const std::chrono::system_clock::time_point& receive_time) {
    runtime_status_.frame_communication_state = frame_communication_state;
    const bool success = runtime_status_.frame_communication_state == CommunicationState::SUCCESS_READ ? true : false;

    updateRebootCounter(frame_id, success, runtime_status_.last_frame_id);
    updateLastFrameStatus(frame_id, success, request_time, receive_time);
    updateFrameDrop(frame_id, success);
  }

  /**
   * @brief Updates the scan pattern and the status of communication for scan pattern watch
   *
   * @param scan_pattern the scan pattern to update
   * @param scan_pattern_communication_state the status of communication with scan pattern watch
   */
  void updateScanPattern(const std::optional<Qb2ScanPattern> scan_pattern,
                         const CommunicationState scan_pattern_communication_state) {
    runtime_status_.scan_pattern_communication_state = scan_pattern_communication_state;
    if (qb2::hasQb2CommunicationFailed(scan_pattern_communication_state) == false && scan_pattern.has_value() == true) {
      scan_pattern_ = scan_pattern.value();
    }
  }

  /**
   * @brief Get the Target Frame Rate
   *
   * @return float
   */
  float getTargetFrameRate() {
    if (qb2_.snapshot.mode == true) {
      return qb2_.snapshot.frame_rate;
    } else {
      return scan_pattern_.frame_rate;
    }
  }

  /**
   * @brief Set flags when disconnecting from Qb2
   *
   */
  void disconnect() {
    runtime_status_.last_frame_success = false;
    runtime_status_.frame_communication_state = CommunicationState::DISCONNECTED;
    runtime_status_.scan_pattern_communication_state = CommunicationState::DISCONNECTED;
  }

  /**
   * @brief Check if the last frame is too old based on target frame rate
   *
   * @return true
   * @return false
   */
  bool isFrameTooOld() {
    if (qb2::hasQb2CommunicationFailed(runtime_status_.frame_communication_state) == true ||
        runtime_status_.frame_communication_state == CommunicationState::NOT_DEFINED ||
        runtime_status_.frame_communication_state == CommunicationState::DISCONNECTED) {
      return false;
    }

    auto duration_since_last_frame = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() -
                                                                   runtime_status_.last_receive_frame_time)
                                         .count();

    static constexpr float allowed_dropped_frames = 5;
    float allowed_duration_since_last_frame_in_seconds = allowed_dropped_frames / getTargetFrameRate();
    if (duration_since_last_frame > allowed_duration_since_last_frame_in_seconds) {
      return true;
    }
    return false;
  }

  /**
   * @brief Updates status for the ros diagnostic updater
   *
   * @param[in, out] status the status to update in diagnostic updater
   */
  void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
    switch (runtime_status_.frame_communication_state) {
      case CommunicationState::NOT_DEFINED:
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Connecting");
        break;
      case CommunicationState::SUCCESS_READ:
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
        break;
      case CommunicationState::SUCCESS_OPEN_STREAM:
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for data");
        break;
      case CommunicationState::FAIL_READ:
      case CommunicationState::FAIL_NO_CONNECTION:
      case CommunicationState::FAIL_EXCEPTION:
      case CommunicationState::DISCONNECTED:
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Reconnecting");
        break;
      case CommunicationState::FAIL_AUTHENTICATION:
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Authentication failed");
        break;
      default:
        break;
    }

    status.add("device_fqdn", qb2_.hostName());

    status.add("last_frame_success", runtime_status_.last_frame_success);
    status.add("last_frame_duration", runtime_status_.last_frame_duration);

    /// HINT: if frame rate is not positive means the scan pattern has not been set yet
    if (scan_pattern_.frame_rate > 0) {
      status.add("horizontal_field_of_view", scan_pattern_.horizontal_field_of_view);
      status.add("vertical_field_of_view", scan_pattern_.vertical_field_of_view);
      status.add("scanlines_up", scan_pattern_.scanlines_up);
      status.add("scanlines_down", scan_pattern_.scanlines_down);
      status.add("angle_spacing", scan_pattern_.angle_spacing);
      status.add("scan_pattern_frame_rate", scan_pattern_.frame_rate);
    }

    status.add("total_frames_published", runtime_status_.total_frames_published);
    status.add("total_frames_dropped", runtime_status_.total_frames_dropped);

    status.add("total_reboots", runtime_status_.total_reboots);
  }

 private:
  /**
   * @brief Calculate the number of detected reboots of the Qb2
   *
   * @param[in] frame_id the counter of the frame received from Qb2
   * @param[in] success the status of the request
   * @param[in] last_frame_id the counter of the last frame received from Qb2
   */
  void updateRebootCounter(uint64_t frame_id, bool success, uint64_t last_frame_id) {
    const bool qb2_rebooted = (frame_id < last_frame_id) && success;
    if (qb2_rebooted == true) {
      runtime_status_.total_reboots++;
    }
  }

  /**
   * @brief Calculates the last frame request duration and set the status
   *
   * @param[in] frame_id the counter of the frame of Qb2
   * @param[in] success the status of the request
   * @param[in] request_time the time of the sending the request
   * @param[in] receive_time the time of the receiving the response
   */
  void updateLastFrameStatus(uint64_t frame_id, bool success, const std::chrono::system_clock::time_point& request_time,
                             const std::chrono::system_clock::time_point& receive_time) {
    runtime_status_.last_receive_frame_time = receive_time;
    runtime_status_.last_frame_duration = std::chrono::duration<double>(receive_time - request_time).count();
    runtime_status_.last_frame_success = success;
    if (runtime_status_.last_frame_id == 0) {
      runtime_status_.last_frame_id = frame_id;
    }
  }

  /**
   * @brief Calculate and set the number of dropped frames
   *
   * @param[in] frame_id the counter of the frame received from Qb2
   * @param[in] success the status of the request
   */
  void updateFrameDrop(uint64_t frame_id, bool success) {
    if (success == true) {
      /// HINT: in stream mode we can calculate the dropped frames after receiving the first frame
      if (frame_id > runtime_status_.last_frame_id && qb2_.snapshot.mode == false) {
        /// HINT: if the difference between frame ids is 1 then we have not dropped any frames
        /// HINT: in stream mode we can only identify a dropped frame after receiving a successful frame
        runtime_status_.total_frames_dropped += frame_id - runtime_status_.last_frame_id - 1;
      }
      runtime_status_.last_frame_id = frame_id;
    } else {
      if (qb2_.snapshot.mode == true) {
        runtime_status_.total_frames_dropped++;
      }
    }
  }

  Qb2Info qb2_;
  Qb2RuntimeStatus runtime_status_;
  Qb2ScanPattern scan_pattern_;
};
}  // namespace ros_interop
}  // namespace blickfeld
