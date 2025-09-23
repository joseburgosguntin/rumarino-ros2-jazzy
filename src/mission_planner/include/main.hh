#include "interfaces/msg/map.hpp"
#include "vision_msgs/msg/bounding_box3_d.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using BoundingBox3D = vision_msgs::msg::BoundingBox3D;
using MapMsg = interfaces::msg::Map;
using OdometryMsg = nav_msgs::msg::Odometry;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

// the object's "class"
enum class ObjectCls {
  Cube = 0,
  Rectangle = 1,
  Gate = 2,
  Shark = 3,
};
const int OBJECT_COUNT = 3;

const float CLOSE_ENOUGH = 0.5; // how close to a waypoint is good
const float OVERSHOOT = 4;      // how much to overshoot crossing a gate
const float FAR_ENOUGH = 10;    // for submarine not hit some other bounding box
const glm::mat2 CLOCKWISE_90_DEG{0, 1, -1, 0};
const glm::mat2 COUNTER_CLOCKWISE_90_DEG{0, -1, 1, 0};

const float CLOSE_ENOUGH_TO_TORPEDO =
    10; // for submarine not hit some other bounding box

const auto TORPEDO_FIRE_DURATION =
    1s; // time to keep torpedo motors at fire PWM

const int TORPEDO_FIRE = (1800 - 1000) / 1000; // Value to fire torpedo

const int DEPTH_ADJUST = 50 / 1000;    // PWM adjustment for depth
const int ROTATION_ADJUST = 50 / 1000; // PWM adjustment for rotation
const int LINEAR_ADJUST = 50 / 1000;   // PWM adjustment for linear movement
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
// TODO: make motor id's zero indexed
// TODO: call everything thrusters not motors if that make sense
const int DEPTH_MOTORS_ID[] = {2, 7};
const int FRONT_MOTORS_ID[] = {1, 5};
const int BACK_MOTORS_ID[] = {4, 8};
const int TORPEDO_MOTORS_ID[] = {3, 6};

namespace step {
struct Shoot {
  int which_torpedo;
};
struct LookAt {
  glm::vec3 pos;
};
struct Navigate {
  glm::vec3 pos;
  Navigate(glm::vec3 p) : pos(p) {}
};
} // namespace step

using Step = std::variant<step::Shoot, step::LookAt, step::Navigate>;

struct Pose {
  glm::vec3 pos;
  glm::quat rot;
};

struct Moving {
  bool depth, rotation, linear;
};

class MissionPlanner : public rclcpp::Node {
public:
  MissionPlanner();

  rclcpp::Subscription<MapMsg>::SharedPtr map_sub;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odometry_sub;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr thrusters_pub;

private:
  bool scouting = true;

  // for now an array to represent sequencial tasks
  ObjectCls tasks[2] = {ObjectCls::Gate, ObjectCls::Cube};
  int next_task_idx = 0;

  std::vector<Step> steps;
  int next_step_idx = 0;

  float thruster_values[TOTAL_THRUSTERS];
  Moving moving;
  Pose sub_pose;
  MapMsg map_cache;

  void handle_map_msg(const MapMsg::SharedPtr map);
  void handle_odometry_msg(const OdometryMsg::SharedPtr imu);

  void plan_and_start_task(ObjectCls object, BoundingBox3D pos);

  // figure out how to make this into `start_next_step` without doing weird
  // recursion
  void send_step(Step step);

  bool navigate(glm::vec3 target_point);
  bool fire_torpedo(int torpedo_number);
  void look_at(glm::vec3 target_point);

  bool adjust_rotation_motors(Pose current_pose, glm::vec3 target_point);
  bool adjust_linear_motors(Pose current_pose, glm::vec3 target_point);
  bool adjust_depth_motors(Pose current_pose, glm::vec3 target_point);
};
