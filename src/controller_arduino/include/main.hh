#include "std_msgs/msg/float64_multi_array.hpp"
#include <cassert>
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
class Controller : public rclcpp::Node {
public:
  Controller();

  rclcpp::Subscription<Float64MultiArray>::SharedPtr thrusters_sub;

private:
  // Serial command format:
  // - T1:value - Thruster 1 with specified PWM value (1000-2000)
  // - T2:value - Thruster 2 with specified PWM value
  // - T3:value - Thruster 3 with specified PWM value
  // - T4:value - Thruster 4 with specified PWM value
  // - D:value  - Depth motors with specified PWM value
  // - P:value  - Torpedo with specified PWM value
  // - C:value  - Camera motor angle (-60 to 60 degrees)
  std::fstream arduino;

  void handle_thrusters_msg(const Float64MultiArray::SharedPtr thrusters);
};
