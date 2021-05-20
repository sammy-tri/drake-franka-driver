#include <array>
#include <chrono>
#include <future>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <common_robotics_utilities/print.hpp>
#include <franka/gripper.h>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

DEFINE_string(robot_ip_address, "", "Address of the shop floor interface");
DEFINE_string(lcm_url, "", "LCM URL for Panda hand driver");
DEFINE_string(lcm_command_channel, "PANDA_HAND_COMMAND",
              "Channel to listen for lcmt_schunk_wsg_command messages on");
DEFINE_string(lcm_status_channel, "PANDA_HAND_STATUS",
              "Channel to publish lcmt_schunk_wsg_status messages on");
DEFINE_int32(control_loop_period_ms, 100, "Control loop period (ms)");
DEFINE_string(
    control_mode, "position", "Choose one from: status_only, position");

namespace anzu {
namespace robot_bridge {
namespace {

constexpr char kModePosition[] = "position";
constexpr char kModeStatusOnly[] = "status_only";

template<typename T>
inline bool IsFutureRunning(const std::future<T>& future) {
  if (future.valid()) {
    const std::future_status status =
        future.wait_for(std::chrono::milliseconds(1));
    return (status != std::future_status::ready);
  } else {
    return false;
  }
}

int64_t CurrentTimeMicroseconds() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string PrintGripperState(const franka::GripperState& state) {
  std::string str_rep =
      "width: " + std::to_string(state.width) + " max width: " +
      std::to_string(state.max_width) + " is_grasped: " +
      std::to_string(state.is_grasped) + " temp: " +
      std::to_string(state.temperature) + " time: " +
      std::to_string(state.time.toSec());
  return str_rep;
}

class PandaHandDriver {
 public:
  PandaHandDriver(
      const std::string& robot_ip_address, const std::string& lcm_url,
      const std::string& lcm_command_channel,
      const std::string& lcm_status_channel,
      const std::string& control_mode)
      : gripper_(robot_ip_address), lcm_(lcm_url),
        lcm_command_channel_(lcm_command_channel),
        lcm_status_channel_(lcm_status_channel),
        control_mode_(control_mode) {
    if (control_mode_ != kModePosition && control_mode_ != kModeStatusOnly) {
      throw std::runtime_error("Invalid --control_mode=" + control_mode_);
    }
    drake::log()->info(
        "Connected to Panda hand version {}", gripper_.serverVersion());

    drake::log()->info("Homing Panda hand...");
    if (control_mode_ != kModeStatusOnly) {
      if (gripper_.homing()) {
        drake::log()->info("...homed Panda hand");

        lcm::Subscription* sub = lcm_.subscribe(
            lcm_command_channel_, &PandaHandDriver::HandleCommandMessage, this);
        sub->setQueueCapacity(5);
      } else {
        throw std::runtime_error("Failed to home Panda hand");
      }
    }

    drake::log()->info(
        "Initial gripper state {}", PrintGripperState(gripper_.readOnce()));
  }

  void ControlLoop(const int32_t control_loop_period_ms) {
    DRAKE_THROW_UNLESS(control_loop_period_ms > 0);
    const std::chrono::milliseconds cycle_duration(control_loop_period_ms);

    const double min_width = 0.0;
    const double max_width = gripper_.readOnce().max_width;
    const double grasp_speed = 0.1;
    const double grasp_force = 50.0;

    while (true) {
      const auto cycle_start = std::chrono::steady_clock::now();

      const auto state = gripper_.readOnce();
      PublishState(state);

      // Poll for incoming command messages.
      while (lcm_.handleTimeout(0) > 0) {}

      if (command_) {
        DRAKE_DEMAND(control_mode_ != kModeStatusOnly);
        // Is there an already running command?
        if (IsFutureRunning(current_operation_)) {
          drake::log()->info("Received new command, stopping existing command");
          gripper_.stop();
          current_operation_.wait();
          drake::log()->info("Existing command stopped");
        } else {
          drake::log()->info("Received new command, no existing command");
        }

        const double current_width = state.width;
        const double target_width =
            common_robotics_utilities::utility::ClampValueAndWarn(
                command_->target_position_mm * 0.001, min_width, max_width);

        auto grasp_operation = [this] (
            const double width, const double speed, const double force,
            const double epsilon_inner, const double epsilon_outer) {
          try {
            drake::log()->info(
                "Starting gripper_.grasp({}, {}, {}, {}, {})",
                width, speed, force, epsilon_inner, epsilon_outer);
            this->gripper_.grasp(
                width, speed, force, epsilon_inner, epsilon_outer);
            drake::log()->info(
                "Finished gripper_.grasp({}, {}, {}, {}, {})",
                width, speed, force, epsilon_inner, epsilon_outer);
          } catch (const std::exception& ex) {
            // Note that even "successful" grasps may cause gripper_.grasp() to
            // time out and/or throw.
            drake::log()->warn(
                "gripper_.grasp({}, {}, {}, {}, {}) failed: {}",
                width, speed, force, epsilon_inner, epsilon_outer, ex.what());
          }
        };

        auto move_operation = [this] (const double width, const double speed) {
          try {
            drake::log()->info("Starting gripper_.move({}, {})", width, speed);
            this->gripper_.move(width, speed);
            drake::log()->info("Finished gripper_.move({}, {})", width, speed);
          } catch (const std::exception& ex) {
            // Note that even "successful" grasps will cause gripper_.move() to
            // time out and throw eventually, since the nominal target position
            // has not been reached.
            drake::log()->warn(
                "gripper_.move({}, {}) failed: {}", width, speed, ex.what());
          }
        };

        if (target_width > current_width) {
          drake::log()->info(
              "Commanding move() target width: {}, current width {}",
              target_width, current_width);
          current_operation_ = std::async(
              std::launch::async, move_operation, target_width, grasp_speed);
        } else if (target_width < current_width) {
          drake::log()->info(
              "Commanding grasp() target width: {}, current width {}",
              target_width, current_width);
          current_operation_ = std::async(
              std::launch::async, grasp_operation, target_width, grasp_speed,
              grasp_force, max_width, max_width);
        } else {
          drake::log()->info(
              "Commanded grasp with == current width, not commanding gripper");
        }

        command_.reset();
      }

      const auto cycle_end = std::chrono::steady_clock::now();
      const auto cycle_elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              cycle_end - cycle_start);

      if (cycle_elapsed < cycle_duration) {
        std::this_thread::sleep_for(cycle_duration - cycle_elapsed);
      }
    }
  }

 private:
  void PublishState(const franka::GripperState& state) {
    status_msg_.utime = CurrentTimeMicroseconds();
    status_msg_.actual_position_mm = state.width * 1000.0;
    // GripperState doesn't include this information.
    status_msg_.actual_force = 0.0;
    if (IsFutureRunning(current_operation_)) {
      // If the gripper is performing an operation, it must be moving. Fake
      // speed value is high enough to convince gripper_bridge that gripper is
      // moving.
      status_msg_.actual_speed_mm_per_s = 5.0;
    } else {
      // If there's no operation, the gripper must not be moving.
      status_msg_.actual_speed_mm_per_s = 0.0;
    }

    lcm_.publish(lcm_status_channel_, &status_msg_);
  }

  void HandleCommandMessage(
      const lcm::ReceiveBuffer*, const std::string&,
      const drake::lcmt_schunk_wsg_command* command) {
    command_ = *command;
  }

  franka::Gripper gripper_;
  lcm::LCM lcm_;
  const std::string lcm_command_channel_;
  const std::string lcm_status_channel_;
  const std::string control_mode_;

  drake::lcmt_schunk_wsg_status status_msg_{};
  std::optional<drake::lcmt_schunk_wsg_command> command_;
  // Gripper commands are blocking, so we run them async'd in a future.
  std::future<void> current_operation_;
};

int DoMain() {
  DRAKE_THROW_UNLESS(FLAGS_robot_ip_address != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_command_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_status_channel != "");

  PandaHandDriver driver(
      FLAGS_robot_ip_address, FLAGS_lcm_url, FLAGS_lcm_command_channel,
      FLAGS_lcm_status_channel, FLAGS_control_mode);
  driver.ControlLoop(FLAGS_control_loop_period_ms);

  return 0;
}
}  // namespace
}  // namespace robot_bridge
}  // namespace anzu

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return anzu::robot_bridge::DoMain();
}
