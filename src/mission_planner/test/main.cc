#include <gtest/gtest.h>
#include <main.hh>

class MissionPlannerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }
};

TEST_F(MissionPlannerTest, HandlesDetectionMessage) {
  auto node = std::make_shared<MissionPlanner>();

  // Publisher to send test detections
  auto pub = node->create_publisher<DetectionsMsg>(
      "/detector/box_detection", 10);

  bool callback_triggered = false;

  // Temporarily wrap the handle function to set flag
  auto sub = node->create_subscription<DetectionsMsg>(
      "/detector/box_detection", 10,
      [&callback_triggered](const DetectionsMsg::SharedPtr) {
        callback_triggered = true;
      });

  // Publish a fake detection
  auto msg = DetectionsMsg();
  pub->publish(msg);

  // Spin until callback triggers or timeout
  auto start = std::chrono::steady_clock::now();
  while (!callback_triggered &&
         std::chrono::steady_clock::now() - start < 2s) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_TRUE(callback_triggered) << "Detection callback was not triggered";
}

TEST_F(MissionPlannerTest, ClientsCanConnect) {
  auto node = std::make_shared<MissionPlanner>();

  // Create dummy action server
  // auto action_server = rclcpp_action::create_server<NavigateAction>(
  //     node,
  //     "navigate_to_waypoint",
  //     [](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateAction::Goal>) {
  //       return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  //     },
  //     [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateAction>> handle) {
  //       auto result = std::make_shared<NavigateAction::Result>();
  //       handle->succeed(result);
  //     },
  //     [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<NavigateAction>>){});

  // Create dummy service server
  auto srv = node->create_service<TorpedoService>(
      "fire_torpedo",
      [](const std::shared_ptr<TorpedoService::Request>,
         std::shared_ptr<TorpedoService::Response>) {
        // no-op
      });

  // Spin for a short while to let clients discover servers
  auto start = std::chrono::steady_clock::now();
  bool action_ready = false, service_ready = false;
  while ((!action_ready || !service_ready) &&
         std::chrono::steady_clock::now() - start < 3s) {
    rclcpp::spin_some(node);
    // action_ready = node->wait_for_action_server(node->navigate_client_, 100ms);
    service_ready = node->torpedo_client->wait_for_service(100ms);
  }

  EXPECT_TRUE(action_ready) << "Navigate action client not connected";
  EXPECT_TRUE(service_ready) << "Torpedo service client not connected";
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
