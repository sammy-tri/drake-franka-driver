#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <common_robotics_utilities/print.hpp>
#include <franka/exception.h>
#include <franka/robot.h>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/lcmt_panda_command.hpp"
#include "drake/lcmt_panda_status.hpp"

DEFINE_string(robot_ip_address, "", "Address of the shop floor interface");
DEFINE_string(lcm_url, "", "LCM URL for Panda driver");
DEFINE_string(lcm_command_channel, "PANDA_COMMAND",
              "Channel to listen for lcmt_panda_command messages on");
DEFINE_string(lcm_status_channel, "PANDA_STATUS",
              "Channel to publish lcmt_panda_status messages on");
DEFINE_double(joint_torque_limit, 100.0, "Joint torque limit");
DEFINE_double(cartesian_force_limit, 100.0, "Cartesian force/torque limit");
DEFINE_string(
    control_mode, "velocity",
    "Choose from: status_only, velocity (default)");
DEFINE_bool(
    latch, false, "Latch previous command if no new command has arrived");
DEFINE_double(
    expire_sec, 0.1,
    "How much delay is allowed for messages to be allowed. Converted to "
    "usec, must be non-negative + finite.");

namespace anzu {
namespace robot_bridge {
namespace {

namespace sp = std::placeholders;

int64_t CurrentTimeMicroseconds() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

// dest, src syntax like memcpy
template <typename T, std::size_t N>
void CopyArrayToVector(std::vector<T>* dest, const std::array<T, N>& src) {
  dest->resize(N);
  memcpy(dest->data(), src.data(), N * sizeof(T));
}

template <typename T, std::size_t N>
std::string PrintArray(const std::string& name, std::array<T, N> data) {
  return
      name + ": " + common_robotics_utilities::print::Print(data, true) + "\n";
}

enum class ControlMode {
  /** Only publish status */
  kStatusOnly,
  /** Command velocities via Joint impedance */
  kVelocity,
};

ControlMode ToControlMode(std::string value) {
  if (value == "status_only") {
    return ControlMode::kStatusOnly;
  } else if (value == "velocity") {
    return ControlMode::kVelocity;
  } else {
    throw std::runtime_error("Invalid ControlMode: " + value);
  }
}

std::string PrintRobotState(const franka::RobotState& state) {
  std::string str_rep = "Panda RobotState:\n";
  str_rep += PrintArray("q", state.q);
  str_rep += PrintArray("q_d", state.q_d);
  str_rep += PrintArray("dq", state.dq);
  str_rep += PrintArray("dq_d", state.dq_d);
  str_rep += PrintArray("ddq_d", state.ddq_d);
  str_rep += PrintArray("theta", state.theta);
  str_rep += PrintArray("dtheta", state.dtheta);
  str_rep += PrintArray("tau_J", state.tau_J);
  str_rep += PrintArray("tau_J_d", state.tau_J_d);
  str_rep += PrintArray("dtau_J", state.dtau_J);
  str_rep += PrintArray("tau_ext_hat_filtered", state.tau_ext_hat_filtered);
  str_rep += PrintArray("joint_contact", state.joint_contact);
  str_rep += PrintArray("joint_contact", state.joint_collision);
  str_rep += PrintArray("cartesian_contact", state.cartesian_contact);
  str_rep += PrintArray("cartesian_collision", state.cartesian_collision);

  str_rep += "errors: " + std::string(state.current_errors) + "\n";
  str_rep += "control success rate: "
          + std::to_string(state.control_command_success_rate) + "\n";
  str_rep += "mode: "
          + std::to_string(static_cast<int>(state.robot_mode)) + "\n";
  str_rep += "time: " + std::to_string(state.time.toSec());

  return str_rep;
}

class PandaDriver {
 public:
  PandaDriver(
      const std::string& robot_ip_address, const std::string& lcm_url,
      const std::string& lcm_command_channel,
      const std::string& lcm_status_channel,
      double joint_torque_limit, double cartesian_force_limit, bool latch,
      uint32_t expire_usec)
      : robot_(robot_ip_address, franka::RealtimeConfig::kIgnore),
        lcm_(lcm_url), lcm_command_channel_(lcm_command_channel),
        lcm_status_channel_(lcm_status_channel), latch_(latch),
        expire_usec_(expire_usec),
        joint_torque_limit_(joint_torque_limit),
        cartesian_force_limit_(cartesian_force_limit) {
    drake::log()->info(
        "Connected to Panda arm version {}", robot_.serverVersion());

    const auto state = robot_.readOnce();
    drake::log()->info("Initial robot state: {}", PrintRobotState(state));

    status_msg_.num_joints = state.q.size();
    status_msg_.joint_position.resize(status_msg_.num_joints, 0.);
    status_msg_.joint_position_desired.resize(status_msg_.num_joints, 0.);
    status_msg_.joint_velocity.resize(status_msg_.num_joints, 0.);
    status_msg_.joint_velocity_desired.resize(status_msg_.num_joints, 0.);
    status_msg_.joint_velocity_desired.resize(status_msg_.num_joints, 0.);
    status_msg_.joint_acceleration_desired.resize(status_msg_.num_joints, 0.);
    status_msg_.joint_torque.resize(status_msg_.num_joints, 0.);
    status_msg_.joint_torque_desired.resize(status_msg_.num_joints, 0.);
    status_msg_.joint_torque_external.resize(status_msg_.num_joints, 0.);

    lcm::Subscription* sub = lcm_.subscribe(
        lcm_command_channel_, &PandaDriver::HandleCommandMessage, this);
    sub->setQueueCapacity(5);
  }

  void ControlLoop(ControlMode mode) {
    try {
      switch (mode) {
        case ControlMode::kStatusOnly: {
          robot_.read(std::bind(&PandaDriver::DoStateRead, this, sp::_1));
          break;
        }
        case ControlMode::kVelocity: {
          DoNonRealtimeControlSetup();
          robot_.control(
            std::bind(&PandaDriver::DoVelocityControl, this, sp::_1, sp::_2),
            franka::ControllerMode::kJointImpedance, true,
            franka::kMaxCutoffFrequency);
          break;
        }
        default: DRAKE_DEMAND(false);
      }
    } catch (const std::exception& ex) {
      drake::log()->error(
          "Control loop exception: [{}]", ex.what());
      command_.reset();
      if (mode != ControlMode::kStatusOnly) {
        drake::log()->error("Running recovery");
        robot_.automaticErrorRecovery();
      }
    }
  }

 private:
  bool DoStateRead(const franka::RobotState& state) {
    PublishRobotState(state);
    return true;
  }

  void DoNonRealtimeControlSetup() {
    // Values originally from setDefaultBehavior in libfranka example code
    std::array<double, 7> joint_torque_limits;
    joint_torque_limits.fill(joint_torque_limit_);

    std::array<double, 6> cartesian_force_limits;
    cartesian_force_limits.fill(cartesian_force_limit_);

    robot_.setCollisionBehavior(
    joint_torque_limits, joint_torque_limits, cartesian_force_limits,
        cartesian_force_limits);
    robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  }

  franka::JointVelocities DoVelocityControl(
      const franka::RobotState& state, franka::Duration)  {
    PublishRobotState(state);

    // Poll for incoming command messages.
    while (lcm_.handleTimeout(0) > 0) {}

    if (!command_ && latch_) {
      command_ = command_prev_;
    }

    franka::JointVelocities v({0, 0, 0, 0, 0, 0, 0});
    if (!command_) {
      return v;
    }

    DRAKE_THROW_UNLESS(
        command_->control_mode_expected ==
        drake::lcmt_panda_status::CONTROL_MODE_VELOCITY);

    const int64_t now = CurrentTimeMicroseconds();
    if (std::abs(now - command_->utime) > expire_usec_) {
      drake::log()->warn("Resetting state velocity command to zero");
      command_.reset();
      command_prev_.reset();
      return v;
    }

    if (command_->num_joints != state.q.size()) {
      throw std::runtime_error(
          "Received command with unexpected number of joints");
    }

    for (int i = 0; i < command_->num_joints; ++i) {
      v.dq[i] = command_->joint_velocity[i];
    }

    command_prev_ = command_;
    return v;
  }

  void HandleCommandMessage(
      const lcm::ReceiveBuffer*, const std::string&,
      const drake::lcmt_panda_command* command) {
    command_ = *command;
  }

  void PublishRobotState(const franka::RobotState& state) {
    status_msg_.utime = CurrentTimeMicroseconds();
    CopyArrayToVector(&status_msg_.joint_position, state.q);
    CopyArrayToVector(&status_msg_.joint_position_desired, state.q_d);
    CopyArrayToVector(&status_msg_.joint_velocity, state.dq);
    CopyArrayToVector(&status_msg_.joint_velocity_desired, state.dq_d);
    CopyArrayToVector(&status_msg_.joint_acceleration_desired, state.ddq_d);
    CopyArrayToVector(&status_msg_.joint_torque, state.tau_J);
    CopyArrayToVector(&status_msg_.joint_torque_desired, state.tau_J_d);
    CopyArrayToVector(
        &status_msg_.joint_torque_external, state.tau_ext_hat_filtered);
    status_msg_.control_command_success_rate =
        state.control_command_success_rate;
    switch (state.robot_mode) {
      case franka::RobotMode::kOther: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kOther;
        break;
      }
      case franka::RobotMode::kIdle: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kIdle;
        break;
      }
      case franka::RobotMode::kMove: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kMove;
        break;
      }
      case franka::RobotMode::kGuiding: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kGuiding;
        break;
      }
      case franka::RobotMode::kReflex: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kReflex;
        break;
      }
      case franka::RobotMode::kUserStopped: {
        status_msg_.robot_mode = drake::lcmt_panda_status::kUserStopped;
        break;
      }
      case franka::RobotMode::kAutomaticErrorRecovery: {
        status_msg_.robot_mode =
            drake::lcmt_panda_status::kAutomaticErrorRecovery;
        break;
      }
    }

    status_msg_.robot_utime = state.time.toMSec() * 1000;
    lcm_.publish(lcm_status_channel_, &status_msg_);
  }

  franka::Robot robot_;
  lcm::LCM lcm_;
  const std::string lcm_command_channel_;
  const std::string lcm_status_channel_;
  const bool latch_{};
  const uint32_t expire_usec_{};
  const double joint_torque_limit_{};
  const double cartesian_force_limit_{};

  drake::lcmt_panda_status status_msg_{};
  std::optional<drake::lcmt_panda_command> command_;
  std::optional<drake::lcmt_panda_command> command_prev_;
};

int DoMain() {
  DRAKE_THROW_UNLESS(FLAGS_robot_ip_address != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_command_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_lcm_status_channel != "");
  DRAKE_THROW_UNLESS(FLAGS_joint_torque_limit > 0.0);
  DRAKE_THROW_UNLESS(FLAGS_cartesian_force_limit > 0.0);
  DRAKE_THROW_UNLESS(
      FLAGS_expire_sec >= 0.0 && std::isfinite(FLAGS_expire_sec));

  const ControlMode mode = ToControlMode(FLAGS_control_mode);

  const uint32_t expire_usec =
      static_cast<uint32_t>(FLAGS_expire_sec * 1e6);
  PandaDriver driver(
      FLAGS_robot_ip_address, FLAGS_lcm_url, FLAGS_lcm_command_channel,
      FLAGS_lcm_status_channel, FLAGS_joint_torque_limit,
      FLAGS_cartesian_force_limit, FLAGS_latch, expire_usec);

  driver.ControlLoop(mode);

  return 0;
}
}  // namespace
}  // namespace robot_bridge
}  // namespace anzu

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return anzu::robot_bridge::DoMain();
}
