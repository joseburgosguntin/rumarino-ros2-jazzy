#include <main.hh>

MissionPlanner::MissionPlanner() : Node("mission_planner") {
  detection_sub = this->create_subscription<DetectionsMsg>(
      "/detector/box_detection", 10,
      [this](const DetectionsMsg::SharedPtr detections) {
        this->handle_dectections_msg(detections);
      });

  navigate_client = rclcpp_action::create_client<NavigateAction>(
      this, "navigate_to_waypoint");

  torpedo_client = this->create_client<TorpedoService>("fire_torpedo");
}

void MissionPlanner::handle_dectections_msg(
    const DetectionsMsg::SharedPtr detections) {
  RCLCPP_INFO(this->get_logger(), "Received: '%s'",
              detections->detector_name.c_str());
  for (auto detection : detections->detections) {
    auto pos =
        glm::vec3(detection.point.x, detection.point.y, detection.point.z);
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

    plan_and_start_task(next_object, pos);
  }
}

void MissionPlanner::handle_navigate_result(NavigateResult result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->success) {
    RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");

    if (scouting) {
      // log.panic("reach waypoint aren't expected while scouting")
    }

    next_step_idx += 1;

    // haven't reach final waypoint (for this object)
    if (next_step_idx < steps.size()) {
      send_step(steps[next_step_idx]);
      // log.debugf("next waypoint: %v", waypoints[next_action_idx])
      return;
    }

    auto next_object = tasks[next_task_idx];

    auto rememebered_pos = rememebered_objets[static_cast<size_t>(next_object)];

    if (rememebered_pos == glm::vec3{}) {
      // log.debug("scouting for %v", next_object)
      scouting = true;
    } else {
      // log.debug("remebered %v postion: %v", next_object,
      // rememebered_pos)

      plan_and_start_task(next_object, rememebered_pos);
    }

  } else {
    RCLCPP_INFO(this->get_logger(), "Navigation failed or canceled");
  }
}

void MissionPlanner::handle_torpedo_response(
    TorpedoService::Response::SharedPtr response) {
  // TODO
}

void MissionPlanner::send_navigate_goal(glm::vec3 pos) {
  // Wait until the action server is available
  if (!navigate_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available");
    return;
  }

  // Create the goal
  auto goal_msg = NavigateAction::Goal();
  goal_msg.target_point.x = pos.x;
  goal_msg.target_point.y = pos.y;
  goal_msg.target_point.z = pos.z;

  // Configure callbacks
  auto send_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_options.feedback_callback =
      [this](auto, NavigateAction::Feedback::ConstSharedPtr feedback) {
        RCLCPP_INFO(this->get_logger(), "Progress: %f%%",
                    feedback->distance_to_target);
      };
  send_options.result_callback = [this](NavigateResult result) {
    this->handle_navigate_result(result);
  };

  // Send the goal asynchronously
  navigate_client->async_send_goal(goal_msg, send_options);
}

void MissionPlanner::send_torpedo_request(int which_torpedo) {
  if (!torpedo_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Service not available");
    return;
  }

  auto request = std::make_shared<TorpedoService::Request>();
  request->torpedo_number = which_torpedo; // 1 or 2

  using ServiceResponseFuture = rclcpp::Client<TorpedoService>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto response = future.get();
    handle_torpedo_response(response);
  };

  torpedo_client->async_send_request(request, response_received_callback);
}

void MissionPlanner::send_look_at_request(glm::vec3 pos) {
  // TODO
}

void MissionPlanner::send_step(Step action) {
  if (std::holds_alternative<step::Shoot>(action)) {
    step::Shoot shoot = std::get<step::Shoot>(action);
    send_torpedo_request(shoot.which_torpedo);
  } else if (std::holds_alternative<step::LookAt>(action)) {
    step::LookAt look_at = std::get<step::LookAt>(action);
    send_look_at_request(look_at.pos);
  } else if (std::holds_alternative<step::Navigate>(action)) {
    step::Navigate navigate = std::get<step::Navigate>(action);
    send_navigate_goal(navigate.pos);
  }
}

void MissionPlanner::plan_and_start_task(Object object, glm::vec3 pos) {
  steps.clear();
  next_step_idx = 0;

  switch (object) {
  case Object::Cube:
  case Object::Rectangle: {
    // Go around
    auto dims = OBJECT_DIMS[static_cast<size_t>(object)];
    // since we don't know which face we are looking at
    auto longest_side = glm::max(dims.x, dims.y);

    auto direction_2d = glm::normalize(glm::vec2{pos} - glm::vec2{sub_pos});
    auto before_2d = glm::vec2{pos} - direction_2d * FAR_ENOUGH;
    auto before = glm::vec3{direction_2d, pos.z};

    auto direction = glm::vec3{direction_2d, 0};

    auto center = pos + direction * (longest_side / 2);
    auto behind = center + direction * (longest_side / 2 + FAR_ENOUGH);
    auto right_2d = glm::vec2{center} + direction_2d * CLOCKWISE_90_DEG *
                                            (longest_side / 2 + FAR_ENOUGH);
    auto right = glm::vec3{right_2d, pos.z};
    auto left_2d = glm::vec2{center} + direction_2d * COUNTER_CLOCKWISE_90_DEG *
                                           (longest_side / 2 + FAR_ENOUGH);
    auto left = glm::vec3{left_2d, pos.z};

    steps.insert(steps.end(), {before, right, behind, left, before});
    break;
  }
  case Object::Gate: {
    // Pass in and overshoot a little
    auto direction_2d = glm::normalize(glm::vec2{pos} - glm::vec2{sub_pos});
    auto before_2d = glm::vec2{pos} - direction_2d * FAR_ENOUGH;
    auto before = glm::vec3{direction_2d, pos.z};

    auto overshoot_2d = glm::vec2{pos} + direction_2d * OVERSHOOT;
    auto overshoot = glm::vec3{overshoot_2d, pos.z};

    steps.insert(steps.end(), {before, overshoot});
    break;
  }
  case Object::Shark: {
    auto direction_2d = glm::normalize(glm::vec2{pos} - glm::vec2{sub_pos});
    auto before_2d = glm::vec2{pos} - direction_2d * CLOSE_ENOUGH_TO_TORPEDO;
    auto before = glm::vec3{direction_2d, pos.z};

    steps.insert(steps.end(), {before, step::LookAt{pos}, step::Shoot{}});
    break;
  }
  }

  next_task_idx += 1;
  scouting = false;
  send_step(steps[next_step_idx]);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
}
