#include <iostream>

#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>

#include <bot_core/joint_state_t.hpp>
#include <bot_core/robot_state_t.hpp>
#include <drake/lcmt_panda_status.hpp>
#include <franka/robot.h>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

DEFINE_string(robot_addr, "192.168.4.2",
              "Address of the shop floor interface");
DEFINE_string(lcm_url, "", "LCM URL for Kuka driver");
DEFINE_string(lcm_robot_state_channel, "EST_ROBOT_STATE",
              "Channel to publish robot_state_t messages on.  "
              "If blank message will not be published.");
DEFINE_string(lcm_command_channel, "PANDA_COMMAND",
              "Channel to listen for commanded joint states.  "
              "Effort and velocity are ignored.");
DEFINE_string(lcm_status_channel, "PANDA_STATUS",
              "Channel to publish lcmt_panda_status messages on.");
DEFINE_string(control_mode, "velocity",
              "Select position or velocity control mode.  "
              "Use of position mode is not recommended.");

namespace franka_driver {

int64_t micros() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

// dest, src syntax like memcpy
template <typename T, std::size_t N>
void CopyArray(std::vector<T>* dest, const std::array<T, N>& src) {
  dest->resize(N);
  if (N == 0) {
    return;
  }

  memcpy(dest->data(), src.data(), N * sizeof(T));
}

template <typename T, std::size_t N>
void PrintArray(const std::string& name, std::array<T, N> data) {
  std::cout << name << ": ";
  for (size_t i = 0; i < N - 1; ++i) {
    std::cout << data[i] << ", ";
  }
  std::cout << data[N - 1] << "\n";
}

void PrintRobotState(const franka::RobotState& state) {
    PrintArray("q", state.q);
    PrintArray("q_d", state.q_d);
    PrintArray("dq", state.dq);
    PrintArray("dq_d", state.dq_d);
    PrintArray("ddq_d", state.ddq_d);
    PrintArray("theta", state.theta);
    PrintArray("dtheta", state.dtheta);
    PrintArray("tau_J", state.tau_J);
    PrintArray("tau_J_d", state.tau_J_d);
    PrintArray("dtau_J", state.dtau_J);
    PrintArray("tau_ext_hat_filtered", state.tau_ext_hat_filtered);
    PrintArray("joint_contact", state.joint_contact);
    PrintArray("joint_contact", state.joint_collision);
    PrintArray("cartesian_contact", state.cartesian_contact);
    PrintArray("cartesian_collision", state.cartesian_collision);

    std::cout << "errors: " << std::string(state.current_errors) << "\n";
    std::cout << "control success rate: " << state.control_command_success_rate << "\n";
    std::cout << "mode: " << static_cast<int>(state.robot_mode) << "\n";
    std::cout << "time: " << state.time.toSec() << "\n";
}

class FrankaDriver {
 public:
  FrankaDriver()
      : robot_(FLAGS_robot_addr, franka::RealtimeConfig::kIgnore),
        lcm_(FLAGS_lcm_url) {

    // Values originally from setDefaultBehavior in libfranka example code
    robot_.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});
    robot_.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

    const auto state = robot_.readOnce();
    initial_q_ = state.q;
    PrintRobotState(state);

    robot_state_msg_.num_joints = state.q.size();
    if (state.q.size() != 7) {
      throw std::runtime_error("Got unexpected number of joints!");
    }

    robot_state_msg_.joint_name.push_back("panda_joint1");
    robot_state_msg_.joint_name.push_back("panda_joint2");
    robot_state_msg_.joint_name.push_back("panda_joint3");
    robot_state_msg_.joint_name.push_back("panda_joint4");
    robot_state_msg_.joint_name.push_back("panda_joint5");
    robot_state_msg_.joint_name.push_back("panda_joint6");
    robot_state_msg_.joint_name.push_back("panda_joint7");
    robot_state_msg_.joint_position.resize(robot_state_msg_.num_joints, 0.);
    robot_state_msg_.joint_velocity.resize(robot_state_msg_.num_joints, 0.);
    robot_state_msg_.joint_effort.resize(robot_state_msg_.num_joints, 0.);

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
        FLAGS_lcm_command_channel,
        &FrankaDriver::HandleCommandMessage, this);
    // Only pay attention to the latest command.
    sub->setQueueCapacity(5);
  }

  franka::JointPositions DoPositionControl(const franka::RobotState& state,
                                           franka::Duration)  {
    if (ticks_++ > 1000) {
      // TODO(sammy-tri) Publish robot state via LCM.
      PrintRobotState(state);
      ticks_ = 0;
    }

    PublishRobotState(state);

    // Poll for incoming command messages.
    while (lcm_.handleTimeout(0) > 0) {}

    if (!command_) {
      return franka::JointPositions(initial_q_);
    }

    // TODO(sammy-tri) Add support for velocity or torque control.
    if (command_->num_joints != state.q.size()) {
      throw std::runtime_error(
          "Received command with unexpected number of joints");
    }

    franka::JointPositions positions({0, 0, 0, 0, 0, 0, 0});
    for (int i = 0; i < command_->num_joints; ++i) {
      positions.q[i] = command_->joint_position[i];
    }
    return positions;
  }

  franka::JointVelocities DoVelocityControl(const franka::RobotState& state,
                                            franka::Duration)  {
    if (ticks_++ > 1000) {
      // TODO(sammy-tri) Publish robot state via LCM.
      PrintRobotState(state);
      ticks_ = 0;
    }

    PublishRobotState(state);

    // Poll for incoming command messages.
    while (lcm_.handleTimeout(0) > 0) {}

    franka::JointVelocities v({0, 0, 0, 0, 0, 0, 0});
    if (!command_) {
      return v;
    }

    const int64_t now = micros();
    if (std::abs(now - command_->utime) > 100000) {
      std::cout << "Resetting stale command\n";
      command_.reset();
      return v;
    }

    // TODO(sammy-tri) Add support for velocity or torque control.
    if (command_->num_joints != state.q.size()) {
      throw std::runtime_error(
          "Received command with unexpected number of joints");
    }

    for (int i = 0; i < command_->num_joints; ++i) {
      v.dq[i] = command_->joint_velocity[i];
    }
    return v;
  }

  void ControlLoop() {
    if (FLAGS_control_mode == "position") {
      robot_.control(std::bind(&FrankaDriver::DoPositionControl, this,
                               std::placeholders::_1, std::placeholders::_2));
    } else if (FLAGS_control_mode == "velocity") {
      robot_.control(std::bind(&FrankaDriver::DoVelocityControl, this,
                               std::placeholders::_1, std::placeholders::_2),
                     franka::ControllerMode::kJointImpedance,
                     true, franka::kMaxCutoffFrequency);
    } else {
      throw std::runtime_error("Unknown control mode: " + FLAGS_control_mode);
    }

  }

 private:
  void HandleCommandMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const bot_core::joint_state_t* command) {
    command_ = *command;
  }

  void PublishRobotState(const franka::RobotState& state) {
    const int64_t now = micros();

    if (!FLAGS_lcm_robot_state_channel.empty()) {
      robot_state_msg_.utime = now;
      for (int i = 0; i < robot_state_msg_.num_joints; ++i) {
        robot_state_msg_.joint_position[i] = state.q[i];
        robot_state_msg_.joint_velocity[i] = state.dq[i];
        robot_state_msg_.joint_effort[i] = state.tau_J[i];
      }
      lcm_.publish(FLAGS_lcm_robot_state_channel, &robot_state_msg_);
    }

    status_msg_.utime = now;
    CopyArray(&status_msg_.joint_position, state.q);
    CopyArray(&status_msg_.joint_position_desired, state.q_d);
    CopyArray(&status_msg_.joint_velocity, state.dq);
    CopyArray(&status_msg_.joint_velocity_desired, state.dq_d);
    CopyArray(&status_msg_.joint_acceleration_desired, state.ddq_d);
    CopyArray(&status_msg_.joint_torque, state.tau_J);
    CopyArray(&status_msg_.joint_torque_desired, state.tau_J_d);
    CopyArray(&status_msg_.joint_torque_external, state.tau_ext_hat_filtered);
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
    lcm_.publish(FLAGS_lcm_status_channel, &status_msg_);
  }

  franka::Robot robot_;
  lcm::LCM lcm_;
  std::array<double, 7> initial_q_;
  int ticks_{0};
  bot_core::robot_state_t robot_state_msg_{};
  drake::lcmt_panda_status status_msg_{};
  std::optional<bot_core::joint_state_t> command_;
};

int do_main() {
  FrankaDriver driver;
  driver.ControlLoop();

  return 0;
}

}  // namespace franka_driver


int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return franka_driver::do_main();
}
