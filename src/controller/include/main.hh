#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/action/navigate_to_waypoint.hpp"
#include "interfaces/srv/fire_torpedo.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
using NavigateAction = interfaces::action::NavigateToWaypoint;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Twist = geometry_msgs::msg::Twist;
using Point = geometry_msgs::msg::Point;
using Int16MultiArray = std_msgs::msg::Int16MultiArray;
using Float32 = std_msgs::msg::Float32;
using TorpedoService = interfaces::srv::FireTorpedo;
using NavigateGoalHandle =
    std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateAction>>;

const int PWM_NEUTRAL = 1500;      // Neutral position
const int PWM_MIN = 1300;          // Max reverse
const int PWM_MAX = 1700;          // Max forward
const int PWM_TORPEDO_FIRE = 1800; // Value to fire torpedo

const int DEPTH_PWM_ADJUST = 50;    // PWM adjustment for depth
const int ROTATION_PWM_ADJUST = 50; // PWM adjustment for rotation
const int LINEAR_PWM_ADJUST = 50;   // PWM adjustment for linear movement
const float DISTANCE_THRESHOLD = 0.1;

// ASCII representation of the position of the thrusters:
//
//  1 *            * 5
//     \          /
//      |________|
//  2*--|        |--* 6
//      |        |
//      |        |
//  3*--|________|--* 7
//      |        |
//  4 */          \* 8
//
const int TOTAL_THRUSTERS = 8;
const int DEPTH_MOTORS_ID[] = {2, 7};
const int FRONT_MOTORS_ID[] = {1, 5};
const int BACK_MOTORS_ID[] = {4, 8};
const int TORPEDO_MOTORS_ID[] = {3, 6};

const auto TORPEDO_FIRE_DURATION =
    1s; // time to keep torpedo motors at fire PWM

struct Pose {
  glm::vec3 pos;
  glm::quat rot;
};

class Controller : public rclcpp::Node {
public:
  Controller();

  rclcpp_action::Server<NavigateAction>::SharedPtr navigate_server;
  rclcpp::Service<TorpedoService>::SharedPtr torpedo_server;
  rclcpp::Subscription<PoseStamped>::SharedPtr zed2i_sub;

  // Add publishers for controller state monitoring
  rclcpp::Publisher<Point>::SharedPtr target_point_pub;
  rclcpp::Publisher<Int16MultiArray>::SharedPtr moving_state_pub;
  rclcpp::Publisher<Float32>::SharedPtr target_distance_pub;
  rclcpp::Publisher<Float32>::SharedPtr original_target_distance_pub;
  rclcpp::Publisher<Float32>::SharedPtr distance_from_start_pub;

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

  int thruster_values[TOTAL_THRUSTERS];
  Pose sub_pose;
  glm::quat sub_rot;
  bool sub_pos_set;
  glm::vec3 target_point;
  bool target_point_set;
  struct {
    bool depth, rotation, linear;
  } moving;

  rclcpp_action::GoalResponse
  handle_navigate_goal(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const NavigateAction::Goal> goal);
  rclcpp_action::CancelResponse
  handle_navigate_cancel(NavigateGoalHandle goal_handle);
  void handle_navigate_accept(NavigateGoalHandle goal_handle);

  void handle_torpedo_request(TorpedoService::Request::SharedPtr request,
                              TorpedoService::Response::SharedPtr response);

  void handle_zed2i_msg(const PoseStamped::SharedPtr pose_stamped);

  bool adjust_depth_motors(Pose current_pose, glm::vec3 target_point);
  bool adjust_rotation_motors(Pose current_pose, glm::vec3 target_point);
  bool adjust_linear_motors(Pose current_pose, glm::vec3 target_point);
};
