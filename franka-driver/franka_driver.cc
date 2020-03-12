#include <iostream>

#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>

#include <bot_core/joint_state_t.hpp>
#include <bot_core/robot_state_t.hpp>
#include <franka/robot.h>
#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

DEFINE_string(robot_addr, "192.168.4.2",
              "Address of the shop floor interface");
DEFINE_string(lcm_url, "", "LCM URL for Kuka driver");
DEFINE_string(lcm_robot_state_channel, "EST_ROBOT_STATE",
              "Channel to publish robot_state_t messages on.  "
              "If blank message will not be published.");
DEFINE_string(lcm_command_channel, "FRANKA_COMMAND",
              "Channel to listen for commanded joint states.  "
              "Effort and velocity are ignored.");
DEFINE_string(control_mode, "velocity",
              "Select position or velocity control mode.  "
              "Use of position mode is not recommended.");
namespace franka_driver {

int64_t micros() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
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

    state_msg_.num_joints = state.q.size();
    if (state.q.size() != 7) {
      throw std::runtime_error("Got unexpected number of joints!");
    }

    state_msg_.joint_name.push_back("panda_joint1");
    state_msg_.joint_name.push_back("panda_joint2");
    state_msg_.joint_name.push_back("panda_joint3");
    state_msg_.joint_name.push_back("panda_joint4");
    state_msg_.joint_name.push_back("panda_joint5");
    state_msg_.joint_name.push_back("panda_joint6");
    state_msg_.joint_name.push_back("panda_joint7");
    state_msg_.joint_position.resize(state_msg_.num_joints, 0.);
    state_msg_.joint_velocity.resize(state_msg_.num_joints, 0.);
    state_msg_.joint_effort.resize(state_msg_.num_joints, 0.);

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
      state_msg_.utime = now;
      for (int i = 0; i < state_msg_.num_joints; ++i) {
        state_msg_.joint_position[i] = state.q[i];
        state_msg_.joint_velocity[i] = state.dq[i];
        state_msg_.joint_effort[i] = state.tau_J[i];
      }
      lcm_.publish(FLAGS_lcm_robot_state_channel, &state_msg_);
    }
  }

  franka::Robot robot_;
  lcm::LCM lcm_;
  std::array<double, 7> initial_q_;
  int ticks_{0};
  bot_core::robot_state_t state_msg_{};
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
