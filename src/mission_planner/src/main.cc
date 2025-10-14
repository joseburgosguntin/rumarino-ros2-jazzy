#include <main.hh>

MissionPlanner::MissionPlanner() : Node("mission_planner") {
  map_sub = this->create_subscription<MapMsg>(
      "/hydrus/map", 10,
      [this](const MapMsg::SharedPtr map) { this->handle_map_msg(map); });
  odometry_sub = this->create_subscription<OdometryMsg>(
      "/hydrus/odometry", 10, [this](const OdometryMsg::SharedPtr odometry) {
        this->handle_odometry_msg(odometry);
      });
  thrusters_pub =
      this->create_publisher<Float64MultiArray>("/hydrus/thrusters", 10);
}

void MissionPlanner::handle_map_msg(const MapMsg::SharedPtr map) {
  RCLCPP_ERROR(this->get_logger(), "GOT MAP MSG");
  // remeber the last map msg
  map_cache = *map;
  if (!scouting) {
    // ignore new_objects while in a task
    return;
  }
  int new_objects_count = map->objects.size() - map_objects_count;
  for (int i = map_objects_count; i < new_objects_count; i += 1) {
    auto new_object = map_cache.objects[i];
    auto next_object_cls = tasks[next_task_idx];
    if (static_cast<int>(next_object_cls) == new_object.cls) {
      plan_and_start_task(next_object_cls, new_object.bbox);
    }
  }
  map_objects_count += new_objects_count;
}

void MissionPlanner::handle_odometry_msg(const OdometryMsg::SharedPtr odometry) {
  auto p = odometry->pose.pose.position;
  auto o = odometry->pose.pose.orientation;
  sub_pose = {.pos = {p.x, p.y, p.z},
              .rot = {static_cast<float>(o.x), static_cast<float>(o.y),
                      static_cast<float>(o.z), static_cast<float>(o.w)}};
}

bool MissionPlanner::navigate(glm::vec3 target_point) {
  RCLCPP_INFO(this->get_logger(), "Goal received: (%f, %f, %f)", target_point.x,
              target_point.y, target_point.z);

  bool original_recorded = false;
  double original_distance = std::numeric_limits<double>::infinity();
  glm::vec3 original_point;

  rclcpp::Rate loop_rate(10); // 10 Hz

  // TODO: move this out of main thread
  while (rclcpp::ok()) {
    if (!original_recorded) {
      original_distance = glm::distance(sub_pose.pos, target_point);
      original_point = sub_pose.pos;
      original_recorded = true;
    }

    auto distance_to_target = glm::distance(sub_pose.pos, target_point);
    if (distance_to_target < DISTANCE_THRESHOLD + 0.05) {
      // RCLCPP_INFO(this->get_logger(), "Target reached.");
      // TODO: In real life, it might a be a good idea to make sure
      // submarine stays on point (due to buoyancy)
      this->moving = {.depth = false, .rotation = false, .linear = false};
      return true;
    }

    // TODO: Check for cancel

    this->moving = {0};
    if (adjust_depth_motors(sub_pose, target_point)) {
      this->moving.depth = true;
    } else if (!adjust_rotation_motors(sub_pose, target_point)) {
      this->moving.rotation = true;
    } else if (!adjust_linear_motors(sub_pose, target_point)) {
      this->moving.linear = true;
    }

    Float64MultiArray thursters_msg;
    thursters_msg.data.insert(thursters_msg.data.begin(),
                              std::begin(thruster_values),
                              std::end(thruster_values));
    thrusters_pub->publish(thursters_msg);

    loop_rate.sleep();
  }

  // # In case of an unexpected exit
  // rospy.logwarn("Action server terminated unexpectedly.")
  return false;
}

void MissionPlanner::look_at(glm::vec3 target_point) {
  // TODO:
}

bool MissionPlanner::fire_torpedo(int torpedo_number) {
  assert(torpedo_number >= 1 && torpedo_number <= 2);

  for (auto motor_id : TORPEDO_MOTORS_ID) {
    thruster_values[motor_id - 1] = TORPEDO_FIRE;
  }
  Float64MultiArray thursters_msg_1;
  thursters_msg_1.data.insert(thursters_msg_1.data.begin(),
                              std::begin(thruster_values),
                              std::end(thruster_values));
  thrusters_pub->publish(thursters_msg_1);

  RCLCPP_INFO(this->get_logger(), "Firing torpedo %d", torpedo_number);

  // # Wait for the firing duration
  // TODO: don't sleep in main thread probably
  rclcpp::sleep_for(TORPEDO_FIRE_DURATION);

  for (auto motor_id : TORPEDO_MOTORS_ID) {
    thruster_values[motor_id - 1] = 0;
  }
  Float64MultiArray thursters_msg_2;
  thursters_msg_2.data.insert(thursters_msg_2.data.begin(),
                              std::begin(thruster_values),
                              std::end(thruster_values));
  thrusters_pub->publish(thursters_msg_2);

  RCLCPP_INFO(this->get_logger(), "Torpedo %d fired successfully",
              torpedo_number);

  return true;
}

void MissionPlanner::send_step(Step action) {
  if (std::holds_alternative<step::Shoot>(action)) {
    step::Shoot shoot = std::get<step::Shoot>(action);
    MissionPlanner::fire_torpedo(shoot.which_torpedo);
  } else if (std::holds_alternative<step::LookAt>(action)) {
    step::LookAt look_at = std::get<step::LookAt>(action);
    MissionPlanner::look_at(look_at.pos);
  } else if (std::holds_alternative<step::Navigate>(action)) {
    step::Navigate navigate = std::get<step::Navigate>(action);
    MissionPlanner::navigate(navigate.pos);
  }
}

// how much planning should there be
// because the bbox we get is only the current approximation
// but responding to every single ajustment in the bbox would result in
// the submarine stopping and adjusting its rotation and depth
// also programming the feedback version one is particularly easy
//
// currently I think the best strategy might be to still have the concept of
// steps but have the plan function simply be re ran each time we want to adjust
// to feedback that would mean we would need to store some initial state of when
// each task has its steps originally planned
//
// its still not clear when a replan is appropiate
//
void MissionPlanner::plan_and_start_task(ObjectCls object, BoundingBox3D bbox) {
  steps.clear();
  next_step_idx = 0;

  auto pos = glm::vec3(bbox.center.position.x, bbox.center.position.y,
                       bbox.center.position.z);
  auto size = glm::vec3(bbox.size.x, bbox.size.y, bbox.size.z);
  auto rot = glm::quat(bbox.center.orientation.x, bbox.center.orientation.y,
                       bbox.center.orientation.z, bbox.center.orientation.w);

  auto pos2d = glm::vec2(pos);
  auto size2d = glm::vec2(size);
  auto sub2d = glm::vec2(sub_pose.pos);
  auto sub_rot = sub_pose.rot;

  switch (object) {
  case ObjectCls::Cube:
  case ObjectCls::Rectangle: {
    // Go around

    // corners of a square centered in 0, with area 1
    glm::vec2 square_corners[] = {
        {+0.5, +0.5},
        {+0.5, -0.5},
        {-0.5, -0.5},
        {-0.5, +0.5},
    };

    // absolute distance from any corner of the object
    const float DISTANCE_TO_CORNER = 0.10; // 0.10 m == 10cm
    // a big number so that any corner is always closer
    const float HUGE_NUMBER = 10000000;

    glm::vec3 corner_pluss[4];
    glm::vec2 starting_corner = glm::vec2{HUGE_NUMBER};
    int starting_i = -1;
    int i = 0;
    for (auto square_corner : square_corners) {
      auto rel_corner = glm::vec2(glm::vec3(square_corner * size2d, 0) * rot);
      auto corner_plus =
          pos2d + rel_corner + glm::normalize(rel_corner) * DISTANCE_TO_CORNER;
      corner_pluss[i] = glm::vec3(corner_plus, pos.z);

      if (glm::length(corner_plus - sub2d) < glm::length(starting_corner)) {
        starting_i = i;
        starting_corner = corner_plus;
      }
      i += 1;
    }

    steps.insert(steps.end(), {
                                  corner_pluss[starting_i + 0 % 4],
                                  corner_pluss[starting_i + 1 % 4],
                                  corner_pluss[starting_i + 2 % 4],
                                  corner_pluss[starting_i + 3 % 4],
                                  corner_pluss[starting_i + 0 % 4],
                                  sub_pose.pos,
                              });
    break;
  }
  case ObjectCls::Gate: {
    //   // Pass in and overshoot a little
    //   auto direction_2d =
    //       glm::normalize(glm::vec2{pos} - glm::vec2{sub_pose.pos});
    //   auto before_2d = glm::vec2{pos} - direction_2d * FAR_ENOUGH;
    //   auto before = glm::vec3{direction_2d, pos.z};
    //
    //   auto overshoot_2d = glm::vec2{pos} + direction_2d * OVERSHOOT;
    //   auto overshoot = glm::vec3{overshoot_2d, pos.z};
    //
    //   steps.insert(steps.end(), {before, overshoot});
    break;
  }
  case ObjectCls::Shark: {
    //   auto direction_2d =
    //       glm::normalize(glm::vec2{pos} - glm::vec2{sub_pose.pos});
    //   auto before_2d = glm::vec2{pos} - direction_2d *
    //   CLOSE_ENOUGH_TO_TORPEDO; auto before = glm::vec3{direction_2d, pos.z};
    //
    //   steps.insert(steps.end(), {before, step::LookAt{pos}, step::Shoot{}});
    break;
  }
  }

  next_task_idx += 1;
  scouting = false;
  send_step(steps[next_step_idx]);
}

bool MissionPlanner::adjust_depth_motors(Pose current_pose,
                                         glm::vec3 target_point) {
  auto dz = target_point.z - current_pose.pos.z;
  if (abs(dz) < DISTANCE_THRESHOLD) {
    // # Stop depth movement by setting all depth motors to neutral
    for (auto motor_id : DEPTH_MOTORS_ID) {
      auto idx = motor_id - 1;
      this->thruster_values[idx] = 0;
    }
    RCLCPP_INFO(this->get_logger(),
                "Target depth reached, stopping depth motors");
    return true;
  }

  for (auto motor_id : DEPTH_MOTORS_ID) {
    auto idx = motor_id - 1; // Convert 1-based to 0-based index
    this->thruster_values[idx] = -glm::sign(dz) * DEPTH_ADJUST;
  }
  return false;
}

float normalize_angle(float angle) {
  return glm::mod(angle + glm::pi<float>(), 2.0f * glm::pi<float>()) -
         glm::pi<float>();
}

bool MissionPlanner::adjust_rotation_motors(Pose current_pose,
                                            glm::vec3 target_point) {

  auto current_yaw = glm::eulerAngles(current_pose.rot).z;
  auto dir = target_point - current_pose.pos;
  auto target_yaw = std::atan2(dir.y, dir.x);
  auto dangle = normalize_angle(target_yaw - current_yaw);
  // auto dz = target_point.z - current_pose.pos.z;
  if (glm::abs(dangle) < DISTANCE_THRESHOLD) {
    for (auto motor_id : FRONT_MOTORS_ID) {
      auto idx = motor_id - 1; // # Convert 1-based to 0-based index
      this->thruster_values[idx] = 0;
    }
    for (auto motor_id : BACK_MOTORS_ID) {
      auto idx = motor_id - 1; // # Convert 1-based to 0-based index
      this->thruster_values[idx] = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Rotation aligned with target");
    return true;
  }

  auto sign = glm::sign(dangle);

  for (auto motor_id : FRONT_MOTORS_ID) {
    auto idx = motor_id - 1; // # Convert 1-based to 0-based index
    if (idx == FRONT_MOTORS_ID[0] - 1) {
      this->thruster_values[idx] = -sign * ROTATION_ADJUST;
    } else {
      this->thruster_values[idx] = sign * ROTATION_ADJUST;
    }
  }

  for (auto motor_id : BACK_MOTORS_ID) {
    auto idx = motor_id - 1; // # Convert 1-based to 0-based index
    if (idx == FRONT_MOTORS_ID[0] - 1) {
      this->thruster_values[idx] = -sign * ROTATION_ADJUST;
    } else {
      this->thruster_values[idx] = sign * ROTATION_ADJUST;
    }
  }

  return false;
}

bool MissionPlanner::adjust_linear_motors(Pose current_pose,
                                          glm::vec3 target_point) {
  if (glm::distance(current_pose.pos, target_point) < DISTANCE_THRESHOLD) {
    for (auto motor_id : FRONT_MOTORS_ID) {
      auto idx = motor_id - 1;
      this->thruster_values[idx] = 0;
    }
    for (auto motor_id : BACK_MOTORS_ID) {
      auto idx = motor_id - 1;
      this->thruster_values[idx] = 0;
    }

    RCLCPP_INFO(this->get_logger(),
                "Target reached, stopping forward movement");
    return true;
  }

  for (auto motor_id : FRONT_MOTORS_ID) {
    auto idx = motor_id - 1; // Convert 1-based to 0-based index
    this->thruster_values[idx] = LINEAR_ADJUST;
  }

  for (auto motor_id : BACK_MOTORS_ID) {
    auto idx = motor_id - 1; // Convert 1-based to 0-based index
    this->thruster_values[idx] = LINEAR_ADJUST;
  }

  return false;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlanner>());
  rclcpp::shutdown();
  return 0;
}
