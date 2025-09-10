#include <main.hh>

Controller::Controller() : Node("mission_planner") {
  this->declare_parameter<std::string>("control_port");
  this->declare_parameter<int>("baud_rate", 115200);

  std::string control_port;
  if (!this->get_parameter("control_port", control_port)) {
    RCLCPP_FATAL(this->get_logger(), "control_port not set!");
  }

  int baud_rate;
  if (!this->get_parameter("baud_rate", control_port)) {
    RCLCPP_WARN(this->get_logger(), "baud_rate not set!");
  }

  // TODO actually use the baud_rate
  arduino = std::fstream(control_port);

  navigate_server = rclcpp_action::create_server<NavigateAction>(
      this, "controller_action",
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const NavigateAction::Goal> goal) {
        return this->handle_navigate_goal(uuid, goal);
      },
      [this](NavigateGoalHandle goal_handle) {
        return this->handle_navigate_cancel(goal_handle);
      },
      [this](NavigateGoalHandle goal_handle) {
        return this->handle_navigate_accept(goal_handle);
      });

  torpedo_server = this->create_service<TorpedoService>(
      "fire_torpedo", [this](TorpedoService::Request::SharedPtr request,
                             TorpedoService::Response::SharedPtr response) {
        this->handle_torpedo_request(request, response);
      });

  zed2i_sub = this->create_subscription<PoseStamped>(
      "/detector/box_detection", 10,
      [this](const PoseStamped::SharedPtr detections) {
        this->handle_zed2i_msg(detections);
      });

  target_point_pub =
      this->create_publisher<Point>("/controller/target_point", 10);
  moving_state_pub =
      this->create_publisher<Int16MultiArray>("/controller/moving_state", 10);
  target_distance_pub =
      this->create_publisher<Float32>("/controller/target_distance", 10);
  original_target_distance_pub = this->create_publisher<Float32>(
      "/controller/original_target_distance", 10);
  distance_from_start_pub =
      this->create_publisher<Float32>("/controller/distance_from_start", 10);
}

rclcpp_action::GoalResponse Controller::handle_navigate_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const NavigateAction::Goal> goal) {

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
Controller::handle_navigate_cancel(NavigateGoalHandle goal_handle) {
  // TODO: do cancel stuff
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Controller::handle_navigate_accept(NavigateGoalHandle goal_handle) {
  const auto goal = goal_handle->get_goal();
  this->target_point = {goal->target_point.x, goal->target_point.y,
                        goal->target_point.z};
  RCLCPP_INFO(this->get_logger(), "Goal received: (%f, %f, %f)", target_point.x,
              target_point.y, target_point.z);

  auto feedback = std::make_shared<NavigateAction::Feedback>();
  auto result = std::make_shared<NavigateAction::Result>();

  bool original_recorded = false;
  double original_distance = std::numeric_limits<double>::infinity();
  glm::vec3 original_point;

  rclcpp::Rate loop_rate(10); // 10 Hz

  // TODO: move this out of main thread
  while (rclcpp::ok()) {
    if (!sub_pos_set) {
      // RCLCPP_WARN(this->get_logger(), "Submarine pose not available yet.");
      loop_rate.sleep();
      continue;
    }

    if (!original_recorded) {
      original_distance = glm::distance(sub_pose.pos, target_point);
      original_point = sub_pose.pos;
      original_recorded = true;
    }

    Float32 original_distance_msg;
    original_distance_msg.data = original_distance;
    original_target_distance_pub->publish(original_distance_msg);

    Float32 distance_from_start_msg;
    distance_from_start_msg.data = glm::distance(sub_pose.pos, original_point);
    distance_from_start_pub->publish(distance_from_start_msg);

    feedback->distance_to_target = glm::distance(sub_pose.pos, target_point);
    goal_handle->publish_feedback(feedback);

    if (feedback->distance_to_target < DISTANCE_THRESHOLD + 0.05) {
      // RCLCPP_INFO(this->get_logger(), "Target reached.");
      result->success = true;
      goal_handle->succeed(result);
      // //         # TODO: In real life, it might a be a good idea to make sure
      // submarine stays on point (due to buoyancy)
      this->moving = {.depth = false, .rotation = false, .linear = false};
      return;
    }

    // Check for cancel
    if (goal_handle->is_canceling()) {
      // RCLCPP_INFO(this->get_logger(), "Goal canceled by client.");
      result->success = false; // I guess?
      goal_handle->canceled(result);
      return;
    }

    this->moving = {0};
    if (adjust_depth_motors(sub_pose, target_point)) {
      this->moving.depth = true;
    } else if (!adjust_rotation_motors(sub_pose, target_point)) {
      this->moving.rotation = true;
    } else if (!adjust_linear_motors(sub_pose, target_point)) {
      this->moving.linear = true;
    }

    for (int i = 0; i < TOTAL_THRUSTERS; i++) {
      arduino << 'T' << i << ':' << thruster_values[i] << '\n';
    }

    // # Depth motors - send average of depth motor values
    int depth_pwm = 0;
    for (int m : DEPTH_MOTORS_ID) {
      depth_pwm += this->thruster_values[m - 1];
    }
    depth_pwm /= std::size(DEPTH_MOTORS_ID);
    arduino << "D:" << depth_pwm << '\n';

    // // TODO: should sending data to torpedo happen here
    // // # Torpedo motors - send average of torpedo motor values
    // int torpedo_pwm = 0;
    // for (int m : TORPEDO_MOTORS_ID) {
    //   torpedo_pwm += this->thruster_values[m - 1];
    // }
    // torpedo_pwm /= TORPEDO_MOTORS_LEN;
    // arduino << "P:" << torpedo_pwm << '\n';

    if (target_point_set) {
      Point target_point_msg;
      target_point_msg.x = this->target_point.x;
      target_point_msg.y = this->target_point.y;
      target_point_msg.z = this->target_point.z;
      target_point_pub->publish(target_point_msg);
    }

    // TODO: just do multiple fields in the msg instead
    Int16MultiArray moving_state_msg;
    moving_state_msg.data = {moving.linear, moving.depth, moving.depth};
    moving_state_pub->publish(moving_state_msg);

    // # Publish delta value (distance to target)
    Float32 target_distance_msg;
    target_distance_msg.data = glm::distance(sub_pose.pos, target_point);
    target_distance_pub->publish(target_distance_msg);

    loop_rate.sleep();
  }

  // # In case of an unexpected exit
  // rospy.logwarn("Action server terminated unexpectedly.")
  result->success = false;
  goal_handle->abort(result);
}

void Controller::handle_torpedo_request(
    TorpedoService::Request::SharedPtr request,
    TorpedoService::Response::SharedPtr response) {
  auto torpedo_number = request->torpedo_number;
  if (!(torpedo_number >= 1 && torpedo_number <= 2)) {
    response->success = false;
    std::ostringstream oss;
    oss << "Invalid torpedo number: " << torpedo_number << ". Must be 1 or 2.";
    std::string msg = oss.str();

    return;
  }

  arduino << "P:" << PWM_NEUTRAL << '\n';
  arduino.flush();
  // rospy.loginfo(f"Firing torpedo {torpedo_number}")

  // # Wait for the firing duration
  // TODO: don't sleep in main thread probably
  rclcpp::sleep_for(TORPEDO_FIRE_DURATION);

  // # Reset to neutral PWM
  arduino << "P:" << PWM_NEUTRAL << '\n';
  arduino.flush();
  // rospy.loginfo(f"Torpedo {torpedo_number} fired successfully")

  response->success = true;
  std::ostringstream oss;
  oss << "Torpedo " << torpedo_number << "fired successfully";
  std::string msg = oss.str();
}

void Controller::handle_zed2i_msg(const PoseStamped::SharedPtr pose_stamped) {
  auto pos = pose_stamped->pose.position;
  auto rot = pose_stamped->pose.orientation;
  sub_pos_set = true;
  sub_pose = Pose{
      .pos = {pos.x, pos.y, pos.z},
      .rot = glm::quat{static_cast<float>(rot.w), static_cast<float>(rot.x),
                       static_cast<float>(rot.y), static_cast<float>(rot.z)}};
}

bool Controller::adjust_depth_motors(Pose current_pose,
                                     glm::vec3 target_point) {
  auto dz = target_point.z - current_pose.pos.z;
  if (abs(dz) < DISTANCE_THRESHOLD) {
    // # Stop depth movement by setting all depth motors to neutral
    for (auto motor_id : DEPTH_MOTORS_ID) {
      auto idx = motor_id - 1;
      this->thruster_values[idx] = PWM_NEUTRAL;
    }
    RCLCPP_INFO(this->get_logger(),
                "Target depth reached, stopping depth motors");
    return true;
  }

  for (auto motor_id : DEPTH_MOTORS_ID) {
    auto idx = motor_id - 1; // Convert 1-based to 0-based index
    this->thruster_values[idx] =
        PWM_NEUTRAL + -glm::sign(dz) * DEPTH_PWM_ADJUST;
  }
  return false;
}

float normalize_angle(float angle) {
  return glm::mod(angle + glm::pi<float>(), 2.0f * glm::pi<float>()) -
         glm::pi<float>();
}

bool Controller::adjust_rotation_motors(Pose current_pose,
                                        glm::vec3 target_point) {

  auto current_yaw = glm::eulerAngles(current_pose.rot).z;
  auto dir = target_point - current_pose.pos;
  auto target_yaw = std::atan2(dir.y, dir.x);
  auto dangle = normalize_angle(target_yaw - current_yaw);
  // auto dz = target_point.z - current_pose.pos.z;
  if (glm::abs(dangle) < DISTANCE_THRESHOLD) {
    for (auto motor_id : FRONT_MOTORS_ID) {
      auto idx = motor_id - 1; // # Convert 1-based to 0-based index
      this->thruster_values[idx] = PWM_NEUTRAL;
    }
    for (auto motor_id : BACK_MOTORS_ID) {
      auto idx = motor_id - 1; // # Convert 1-based to 0-based index
      this->thruster_values[idx] = PWM_NEUTRAL;
    }

    RCLCPP_INFO(this->get_logger(), "Rotation aligned with target");
    return true;
  }

  auto sign = glm::sign(dangle);

  for (auto motor_id : FRONT_MOTORS_ID) {
    auto idx = motor_id - 1; // # Convert 1-based to 0-based index
    if (idx == FRONT_MOTORS_ID[0] - 1) {
      this->thruster_values[idx] = PWM_NEUTRAL - sign * ROTATION_PWM_ADJUST;
    } else {
      this->thruster_values[idx] = PWM_NEUTRAL + sign * ROTATION_PWM_ADJUST;
    }
  }

  for (auto motor_id : BACK_MOTORS_ID) {
    auto idx = motor_id - 1; // # Convert 1-based to 0-based index
    if (idx == FRONT_MOTORS_ID[0] - 1) {
      this->thruster_values[idx] = PWM_NEUTRAL - sign * ROTATION_PWM_ADJUST;
    } else {
      this->thruster_values[idx] = PWM_NEUTRAL + sign * ROTATION_PWM_ADJUST;
    }
  }

  return false;
}

bool Controller::adjust_linear_motors(Pose current_pose,
                                      glm::vec3 target_point) {
  if (glm::distance(current_pose.pos, target_point) < DISTANCE_THRESHOLD) {
    for (auto motor_id : FRONT_MOTORS_ID) {
      auto idx = motor_id - 1;
      this->thruster_values[idx] = PWM_NEUTRAL;
    }
    for (auto motor_id : BACK_MOTORS_ID) {
      auto idx = motor_id - 1;
      this->thruster_values[idx] = PWM_NEUTRAL;
    }

    RCLCPP_INFO(this->get_logger(),
                "Target reached, stopping forward movement");
    return true;
  }

  for (auto motor_id : FRONT_MOTORS_ID) {
    auto idx = motor_id - 1; // Convert 1-based to 0-based index
    this->thruster_values[idx] = PWM_NEUTRAL + LINEAR_PWM_ADJUST;
  }

  for (auto motor_id : BACK_MOTORS_ID) {
    auto idx = motor_id - 1; // Convert 1-based to 0-based index
    this->thruster_values[idx] = PWM_NEUTRAL + LINEAR_PWM_ADJUST;
  }

  return false;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
