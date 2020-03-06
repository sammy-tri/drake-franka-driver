#include <iostream>
#include <stdexcept>

#include <franka/gripper.h>
#include <gflags/gflags.h>

DEFINE_string(robot_addr, "192.168.4.2",
              "Address of the shop floor interface");

namespace {
void PrintGripperState(const franka::GripperState& state) {
  std::cout << "width: " << state.width
            << " max: " << state.max_width
            << " is_grasped: " << state.is_grasped
            << " temp: " << state.temperature
            << " time: " << state.time.toSec()
            << "\n";
}
}  // namespace

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  franka::Gripper hand(FLAGS_robot_addr);
  if (!hand.homing()) {
    throw std::runtime_error("Homing failed.");
  }

  std::cout << "Gripper software version: " << hand.serverVersion() << "\n";

  std::vector<char> buf(256);
  double width = 0.01;
  double speed = 0.1;
  double force = 60;
  double epsilon_inner = 0.005;
  double epsilon_outer = 0.005;
  double max_width = hand.readOnce().max_width;

  while (true) {
    std::cout << "\n";
    PrintGripperState(hand.readOnce());

    std::cin.getline(buf.data(), buf.size());
    std::string cmd(buf.data());

    if (cmd.compare("h") == 0) {
      std::cout << "h  : help\n";
      std::cout << "c,.: close gripper\n";
      std::cout << "o,0: open gripper\n";
      std::cout << "m  : move gripper (to close)\n";
      std::cout << "f  : set grasp force\n";
      std::cout << "s  : set grasp speed\n";
      std::cout << "w  : set grasp finger width\n";
      std::cout << "e  : set epsilon inner\n";
      std::cout << "E  : set epsilon outer\n";
      std::cout << "q  : quit\n";
    } else if (cmd.compare("c") == 0 || cmd.compare(".") == 0) {
      if (!hand.grasp(width, speed, force, epsilon_inner, epsilon_outer)) {
        std::cout << "grasp failed\n";
      }
    } else if (cmd.compare("o") == 0 || cmd.compare("0") == 0) {
      if (!hand.move(max_width, speed)) {
        std::cout << "open failed\n";
      }
    } else if (cmd.compare("m") == 0) {
      if (!hand.move(max_width, speed)) {
        std::cout << "move failed\n";
      }
    } else if (cmd.compare("f") == 0) {
      std::cout << "\ndesired grasp force: ";
      std::cin.getline(buf.data(), buf.size());
      force = std::stod(std::string(buf.data()));
    } else if (cmd.compare("s") == 0) {
      std::cout << "\ndesired grasp speed: ";
      std::cin.getline(buf.data(), buf.size());
      speed = std::stod(std::string(buf.data()));
    } else if (cmd.compare("w") == 0) {
      std::cout << "\ndesired grasp finger width: ";
      std::cin.getline(buf.data(), buf.size());
      width = std::stod(std::string(buf.data()));
    } else if (cmd.compare("e") == 0) {
      std::cout << "\ndesired epsilon inner: ";
      std::cin.getline(buf.data(), buf.size());
      epsilon_inner = std::stod(std::string(buf.data()));
    } else if (cmd.compare("E") == 0) {
      std::cout << "\ndesired epsilon outer: ";
      std::cin.getline(buf.data(), buf.size());
      epsilon_outer = std::stod(std::string(buf.data()));
    } else if (cmd.compare("q") == 0) {
      break;
    }
  }

  return 0;
}
