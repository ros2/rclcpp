// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_set.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

class TestThreadSafeStorage : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("node", "ns");
  }

  std::shared_ptr<rclcpp::Node> node;
};

class TestWaitable : public rclcpp::Waitable
{
public:
  TestWaitable()
  : is_ready_(false) {}
  bool add_to_wait_set(rcl_wait_set_t *) override {return true;}

  bool is_ready(rcl_wait_set_t *) override {return is_ready_;}

  void execute() override {}

  void set_is_ready(bool value) {is_ready_ = value;}

private:
  bool is_ready_;
};

TEST_F(TestThreadSafeStorage, default_construct_destruct) {
  rclcpp::ThreadSafeWaitSet wait_set;
  EXPECT_TRUE(rcl_wait_set_is_valid(&wait_set.get_rcl_wait_set()));

  // Expected behavior of thread-safe is to timeout here
  EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_set.wait(std::chrono::milliseconds(10)).kind());
}

TEST_F(TestThreadSafeStorage, iterables_construct_destruct) {
  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    "topic", 10, [](test_msgs::msg::Empty::SharedPtr) {});
  // This is long, so it can stick around
  auto timer = node->create_wall_timer(std::chrono::seconds(100), []() {});
  auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
  auto service =
    node->create_service<test_msgs::srv::Empty>(
    "service",
    [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {});
  auto client = node->create_client<test_msgs::srv::Empty>("service");
  auto waitable = std::make_shared<TestWaitable>();
  auto subscriptions =
    std::vector<rclcpp::ThreadSafeWaitSet::SubscriptionEntry>{{subscription}};
  auto guard_conditions =
    std::vector<rclcpp::GuardCondition::SharedPtr>{guard_condition};
  auto timers =
    std::vector<rclcpp::TimerBase::SharedPtr>{timer};
  auto clients =
    std::vector<rclcpp::ClientBase::SharedPtr>{client};
  auto services =
    std::vector<rclcpp::ServiceBase::SharedPtr>{service};
  auto waitables =
    std::vector<rclcpp::ThreadSafeWaitSet::WaitableEntry>{{waitable}};
  rclcpp::ThreadSafeWaitSet wait_set(
    subscriptions, guard_conditions, timers, clients, services, waitables);

  EXPECT_TRUE(rcl_wait_set_is_valid(&wait_set.get_rcl_wait_set()));
}

TEST_F(TestThreadSafeStorage, add_remove_dynamically) {
  rclcpp::ThreadSafeWaitSet wait_set;

  // Adds more coverage
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    "topic", 10, [](test_msgs::msg::Empty::SharedPtr) {}, options);

  rclcpp::SubscriptionWaitSetMask mask{true, true, true};
  wait_set.add_subscription(subscription, mask);
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_subscription(subscription, mask),
    std::runtime_error("subscription already associated with a wait set"));
  wait_set.remove_subscription(subscription, mask);

  // This is long, so it can stick around and be removed
  auto timer = node->create_wall_timer(std::chrono::seconds(100), []() {});
  wait_set.add_timer(timer);
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_timer(timer),
    std::runtime_error("timer already in use by another wait set"));
  wait_set.remove_timer(timer);

  auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
  wait_set.add_guard_condition(guard_condition);
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_guard_condition(guard_condition),
    std::runtime_error("guard condition already in use by another wait set"));
  wait_set.remove_guard_condition(guard_condition);

  auto service =
    node->create_service<test_msgs::srv::Empty>(
    "service",
    [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {});
  wait_set.add_service(service);
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_service(service),
    std::runtime_error("service already in use by another wait set"));
  wait_set.remove_service(service);

  auto client = node->create_client<test_msgs::srv::Empty>("service");
  wait_set.add_client(client);
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_client(client),
    std::runtime_error("client already in use by another wait set"));
  wait_set.remove_client(client);

  auto waitable = std::make_shared<TestWaitable>();
  wait_set.add_waitable(waitable);
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_waitable(waitable),
    std::runtime_error("waitable already in use by another wait set"));
  wait_set.remove_waitable(waitable);
  wait_set.prune_deleted_entities();

  // Expected behavior of thread-safe is to timeout here
  EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_set.wait(std::chrono::milliseconds(10)).kind());
}

TEST_F(TestThreadSafeStorage, add_remove_nullptr) {
  rclcpp::ThreadSafeWaitSet wait_set;

  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_subscription(nullptr), std::invalid_argument("subscription is nullptr"));
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.remove_subscription(nullptr), std::invalid_argument("subscription is nullptr"));

  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_guard_condition(nullptr), std::invalid_argument("guard_condition is nullptr"));
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.remove_guard_condition(nullptr), std::invalid_argument("guard_condition is nullptr"));

  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_timer(nullptr), std::invalid_argument("timer is nullptr"));
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.remove_timer(nullptr), std::invalid_argument("timer is nullptr"));

  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_client(nullptr), std::invalid_argument("client is nullptr"));
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.remove_client(nullptr), std::invalid_argument("client is nullptr"));

  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_service(nullptr), std::invalid_argument("service is nullptr"));
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.remove_service(nullptr), std::invalid_argument("service is nullptr"));

  RCLCPP_EXPECT_THROW_EQ(
    wait_set.add_waitable(nullptr), std::invalid_argument("waitable is nullptr"));
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.remove_waitable(nullptr), std::invalid_argument("waitable is nullptr"));
}

TEST_F(TestThreadSafeStorage, add_remove_out_of_scope) {
  rclcpp::ThreadSafeWaitSet wait_set;

  {
    auto subscription = node->create_subscription<test_msgs::msg::Empty>(
      "topic", 10, [](test_msgs::msg::Empty::SharedPtr) {});
    wait_set.add_subscription(subscription);

    // This is short, so if it's not cleaned up, it will trigger wait
    auto timer = node->create_wall_timer(std::chrono::milliseconds(1), []() {});
    wait_set.add_timer(timer);

    auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
    wait_set.add_guard_condition(guard_condition);

    auto service =
      node->create_service<test_msgs::srv::Empty>(
      "service",
      [](
        const test_msgs::srv::Empty::Request::SharedPtr,
        test_msgs::srv::Empty::Response::SharedPtr) {});
    wait_set.add_service(service);

    auto client = node->create_client<test_msgs::srv::Empty>("service");
    wait_set.add_client(client);

    auto waitable = std::make_shared<TestWaitable>();
    wait_set.add_waitable(waitable);
  }

  EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_set.wait(std::chrono::milliseconds(10)).kind());
}

TEST_F(TestThreadSafeStorage, wait_subscription) {
  rclcpp::ThreadSafeWaitSet wait_set;

  // Not added to wait_set, just used for publishing to the topic
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10);

  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    "topic", 10, [](test_msgs::msg::Empty::SharedPtr) {});
  wait_set.add_subscription(subscription);

  {
    auto wait_result = wait_set.wait(std::chrono::milliseconds(10));
    EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_result.kind());
  }

  publisher->publish(test_msgs::msg::Empty());
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    EXPECT_EQ(rclcpp::WaitResultKind::Ready, wait_result.kind());
  }
}

TEST_F(TestThreadSafeStorage, wait_timer) {
  rclcpp::ThreadSafeWaitSet wait_set;

  auto timer = node->create_wall_timer(std::chrono::milliseconds(1), []() {});
  wait_set.add_timer(timer);
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    EXPECT_EQ(rclcpp::WaitResultKind::Ready, wait_result.kind());
  }
}

TEST_F(TestThreadSafeStorage, wait_client_service) {
  rclcpp::ThreadSafeWaitSet wait_set;
  auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
  wait_set.add_guard_condition(guard_condition);

  auto service =
    node->create_service<test_msgs::srv::Empty>(
    "service",
    [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {});
  wait_set.add_service(service);

  auto client = node->create_client<test_msgs::srv::Empty>("service");
  wait_set.add_client(client);
  {
    auto wait_result = wait_set.wait(std::chrono::milliseconds(10));
    EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_result.kind());
  }
  client->async_send_request(std::make_shared<test_msgs::srv::Empty::Request>());
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(1));
    EXPECT_EQ(rclcpp::WaitResultKind::Ready, wait_result.kind());
  }
}

TEST_F(TestThreadSafeStorage, wait_waitable) {
  rclcpp::ThreadSafeWaitSet wait_set;
  auto waitable = std::make_shared<TestWaitable>();
  wait_set.add_waitable(waitable);
  {
    // This waitable doesn't add itself to the rcl_wait_set_t, so Timeout is to be expected
    auto wait_result = wait_set.wait(std::chrono::milliseconds(10));
    EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_result.kind());
  }
}
