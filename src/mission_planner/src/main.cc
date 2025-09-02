#include "interfaces/action/navigate_to_waypoint.hpp"
#include "interfaces/msg/detections.hpp"
#include "interfaces/srv/fire_torpedo.hpp"
#include <glm/glm.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

using namespace std::chrono_literals;
using Detections = interfaces::msg::Detections;
using NavigateAction = interfaces::action::NavigateToWaypoint;
using FireTorpedo = interfaces::srv::FireTorpedo;

enum class Object {
  Cube,
  Rectangle,
  Gate,
};
const int OBJECT_COUNT = 3;

float CLOSE_ENOUGH = 0.5; // how close to a waypoint is good
float OVERSHOOT = 4;      // how much to overshoot crossing a gate
float FAR_ENOUGH = 10;    // for submarine not hit some other bounding box
glm::mat2 CLOCKWISE_90_DEG{0, 1, -1, 0};
glm::mat2 COUNTER_CLOCKWISE_90_DEG{0, -1, 1, 0};
glm::vec3 OBJECT_DIMS[OBJECT_COUNT] = {
    {10, 10, 10}, // Cube
    {10, 20, 10}, // Rectangle
    {},           // Gate
};

class MissionPlanner : public rclcpp::Node {
public:
  MissionPlanner() : Node("mission_planner") {
    detection_sub_ = this->create_subscription<Detections>(
        "/detector/box_detection", 10,
        [this](const Detections::SharedPtr detections) {
          RCLCPP_INFO(this->get_logger(), "Received: '%s'",
                      detections->detector_name.c_str());
          for (auto detection : detections->detections) {
            auto pos = glm::vec3(detection.point.x, detection.point.y,
                                 detection.point.z);
            // TODO: figure out how to get object from detection object
            auto object = Object::Cube;

            // log.debugf("found %v at %v", object, pos)
            // always remeber object that are found
            // should the remebered position instead be that pos after we
            // transform it to center of object, cuz it would be a point that
            // keeps moving around
            rememebered_objets[static_cast<size_t>(object)] = pos;

            if (!scouting) {
              // log.warnf("ignoring %v: not scouting", object)
              return;
            }

            auto next_object = tasks[next_task_idx];
            if (next_object != object) {
              // log.warnf("ignoring %v: not next task %v", object, next_object)
              return;
            }

            plan(next_object, pos);
            next_task_idx += 1;
            scouting = false;
            send_navigate_goal(waypoints[next_waypoint_idx]);
          }
        });

    navigate_client_ = rclcpp_action::create_client<NavigateAction>(
        this, "navigate_to_waypoint");

    torpedo_client_ = this->create_client<FireTorpedo>("fire_torpedo");
  }

private:
  rclcpp::Subscription<Detections>::SharedPtr detection_sub_;
  rclcpp_action::Client<NavigateAction>::SharedPtr navigate_client_;
  rclcpp::Client<FireTorpedo>::SharedPtr torpedo_client_;

  glm::vec3 sub_pos = {0, 0, 0};
  bool scouting = true;
  // maybe switch this to a list of faces that were detected
  glm::vec3 rememebered_objets[OBJECT_COUNT] = {};

  // for now an array to represent sequencial tasks
  Object tasks[2] = {Object::Gate, Object::Cube};
  int next_task_idx = 0;

  // TODO: this can later become a list of actions
  std::vector<glm::vec3> waypoints;
  int next_waypoint_idx = 0;

  void send_navigate_goal(glm::vec3 pos) {
    // Wait until the action server is available
    if (!navigate_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    // Create the goal
    auto goal_msg = NavigateAction::Goal();
    goal_msg.target_point.x = pos.x;
    goal_msg.target_point.y = pos.y;
    goal_msg.target_point.z = pos.z;

    // Configure callbacks
    auto send_options =
        rclcpp_action::Client<NavigateAction>::SendGoalOptions();
    send_options.feedback_callback =
        [this](auto, NavigateAction::Feedback::ConstSharedPtr feedback) {
          RCLCPP_INFO(this->get_logger(), "Progress: %f%%",
                      feedback->distance_to_target);
        };
    using GoalHandleNavigateAction =
        rclcpp_action::ClientGoalHandle<NavigateAction>;
    send_options.result_callback =
        [this](GoalHandleNavigateAction::WrappedResult result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
              result.result->success) {
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");

            if (scouting) {
              // log.panic("reach waypoint aren't expected while scouting")
            }

            next_waypoint_idx += 1;

            // haven't reach final waypoint (for this object)
            if (next_waypoint_idx < waypoints.size()) {
              // log.debugf("next waypoint: %v", waypoints[next_waypoint_idx])
              return;
            }

            auto next_object = tasks[next_task_idx];

            auto rememebered_pos =
                rememebered_objets[static_cast<size_t>(next_object)];

            if (rememebered_pos == glm::vec3{}) {
              // log.debug("scouting for %v", next_object)
              scouting = true;
            } else {
              // log.debug("remebered %v postion: %v", next_object,
              // rememebered_pos)

              plan(next_object, rememebered_pos);
              next_task_idx += 1;
              scouting = false;
            }

          } else {
            RCLCPP_INFO(this->get_logger(), "Navigation failed or canceled");
          }
        };

    // Send the goal asynchronously
    navigate_client_->async_send_goal(goal_msg, send_options);
  }

  void send_torpedo_request(double x, double y, double z) {
    if (!torpedo_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service not available");
      return;
    }

    auto request = std::make_shared<FireTorpedo::Request>();
    request->torpedo_number = 1; // 1 or 2

    using ServiceResponseFuture = rclcpp::Client<FireTorpedo>::SharedFuture;

    auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Navigation succeeded: %s",
                    response->message.c_str());
        // TODO: remeber to update next_waypoint_idx, even tho at 
        //       that point it should be called next_action_idx
      } else {
        RCLCPP_ERROR(this->get_logger(), "Navigation failed: %s",
                     response->message.c_str());
      }
    };

    torpedo_client_->async_send_request(request, response_received_callback);
  }

  void plan(Object object, glm::vec3 pos) {
    waypoints.clear();
    next_waypoint_idx = 0;

    auto direction_2d = glm::normalize(glm::vec2{pos} - glm::vec2{sub_pos});
    auto before_2d = glm::vec2{pos} - direction_2d * FAR_ENOUGH;
    auto before = glm::vec3{direction_2d, pos.z};

    switch (object) {
    case Object::Cube:
    case Object::Rectangle: {
      // Go around
      auto dims = OBJECT_DIMS[static_cast<size_t>(object)];
      // since we don't know which face we are looking at
      auto longest_side = glm::max(dims.x, dims.y);

      auto direction = glm::vec3{direction_2d, 0};

      auto center = pos + direction * (longest_side / 2);
      auto behind = center + direction * (longest_side / 2 + FAR_ENOUGH);
      auto right_2d = glm::vec2{center} + direction_2d * CLOCKWISE_90_DEG *
                                              (longest_side / 2 + FAR_ENOUGH);
      auto right = glm::vec3{right_2d, pos.z};
      auto left_2d = glm::vec2{center} + direction_2d *
                                             COUNTER_CLOCKWISE_90_DEG *
                                             (longest_side / 2 + FAR_ENOUGH);
      auto left = glm::vec3{left_2d, pos.z};
      waypoints.insert(waypoints.end(), {before, right, behind, left, before});
      break;
    }
    case Object::Gate:
      // Pass in and overshoot a little
      auto overshoot_2d = glm::vec2{pos} + direction_2d * OVERSHOOT;
      auto overshoot = glm::vec3{overshoot_2d, pos.z};
      waypoints.insert(waypoints.end(), {before, overshoot});
      break;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
}
