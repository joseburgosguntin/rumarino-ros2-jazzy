#include "interfaces/action/navigate_to_waypoint.hpp"
#include "interfaces/msg/detections.hpp"
#include "interfaces/srv/fire_torpedo.hpp"
#include <glm/glm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using DetectionsMsg = interfaces::msg::Detections;
using NavigateAction = interfaces::action::NavigateToWaypoint;
using TorpedoService = interfaces::srv::FireTorpedo;
// using LookAtService = interfaces::srv::LookAt;
using NavigateResult =
    rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult;

enum class Object {
  Cube,
  Rectangle,
  Gate,
  Shark,
};
const int OBJECT_COUNT = 3;

const float CLOSE_ENOUGH = 0.5; // how close to a waypoint is good
const float OVERSHOOT = 4;      // how much to overshoot crossing a gate
const float FAR_ENOUGH = 10;    // for submarine not hit some other bounding box
const glm::mat2 CLOCKWISE_90_DEG{0, 1, -1, 0};
const glm::mat2 COUNTER_CLOCKWISE_90_DEG{0, -1, 1, 0};
const glm::vec3 OBJECT_DIMS[OBJECT_COUNT] = {
    {10, 10, 10}, // Cube
    {10, 20, 10}, // Rectangle
    {},           // Gate
};
const float CLOSE_ENOUGH_TO_TORPEDO = 10;    // for submarine not hit some other bounding box
                                             //

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
}

using Step = std::variant<step::Shoot, step::LookAt, step::Navigate>;

class MissionPlanner : public rclcpp::Node {
public:
  MissionPlanner();

  rclcpp::Subscription<DetectionsMsg>::SharedPtr detection_sub;
  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client;
  rclcpp::Client<TorpedoService>::SharedPtr torpedo_client;
  // rclcpp::Client<LookAtService>::SharedPtr look_at_client;

private:
  glm::vec3 sub_pos = {0, 0, 0};
  bool scouting = true;
  // switch to some sort of object to volume-ish thingy
  glm::vec3 rememebered_objets[OBJECT_COUNT] = {};

  // for now an array to represent sequencial tasks
  Object tasks[2] = {Object::Gate, Object::Cube};
  int next_task_idx = 0;

  std::vector<Step> steps;
  int next_step_idx = 0;

  void handle_dectections_msg(const DetectionsMsg::SharedPtr detections);

  void plan_and_start_task(Object object, glm::vec3 pos);

  void send_navigate_goal(glm::vec3 pos);
  void send_torpedo_request(int which_torpedo);
  void send_look_at_request(glm::vec3 pos);

  // figure out how to make this into `start_next_step` without doing weird recursion
  void send_step(Step step);

  void handle_navigate_result(NavigateResult result);
  void handle_torpedo_response(TorpedoService::Response::SharedPtr response);
  // void handle_look_at_response(LookAtService::Response::SharedPtr response);
};
