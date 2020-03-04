#include <iostream>

#include <gflags/gflags.h>
#include <franka/robot.h>

DEFINE_string(robot_addr, "192.168.4.2",
              "Address of the shop floor interface");

namespace franka_driver {

int do_main() {
  franka::Robot robot(FLAGS_robot_addr, franka::RealtimeConfig::kIgnore);

  // Values originally from setDefaultBehavior in libfranka example code
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

  const auto state = robot.readOnce();

  std::cout << "q: ";
  for (size_t i = 0; i < state.q.size(); ++i) {
    std::cout << state.q[i] << ", ";
  }
  std::cout << "\n";

  return 0;
}

}  // namespace franka_driver


int main(int argc, char** argv) {
  return franka_driver::do_main();
}
