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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_set.hpp"
#include "rclcpp/wait_set_policies/static_storage.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

class TestStaticStorage : public ::testing::Test
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

TEST_F(TestStaticStorage, iterables_construct_destruct) {
  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    "topic", 10, [](test_msgs::msg::Empty::SharedPtr) {});
  // This is long, so it can stick around and be removed
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
  rclcpp::StaticWaitSet<1, 1, 1, 1, 1, 1> wait_set(
    {{{subscription}}}, {guard_condition}, {timer}, {client}, {service}, {{{waitable}}});

  EXPECT_TRUE(rcl_wait_set_is_valid(&wait_set.get_rcl_wait_set()));
}

// Because these StaticWaitSet's have templated sizes larger than the input arguments passed
// to the constructor, their shared-pointer contents will be default constructed to null. This
// test just checks the appropriate exception is thrown.
// std::shared_ptr<StaticWaitSet>::reset() is not required for these exceptions, it just
// disables the unused return value warning of std::make_shared
TEST_F(TestStaticStorage, fixed_storage_needs_pruning) {
  {
    using StaticWaitSet = rclcpp::StaticWaitSet<1, 0, 0, 0, 0, 0>;
    RCLCPP_EXPECT_THROW_EQ(
      std::make_shared<StaticWaitSet>().reset(),
      std::runtime_error("unexpected condition, fixed storage policy needs pruning"));
  }
  {
    using StaticWaitSet = rclcpp::StaticWaitSet<0, 1, 0, 0, 0, 0>;
    RCLCPP_EXPECT_THROW_EQ(
      std::make_shared<StaticWaitSet>().reset(),
      std::runtime_error("unexpected condition, fixed storage policy needs pruning"));
  }
  {
    using StaticWaitSet = rclcpp::StaticWaitSet<0, 0, 1, 0, 0, 0>;
    RCLCPP_EXPECT_THROW_EQ(
      std::make_shared<StaticWaitSet>().reset(),
      std::runtime_error("unexpected condition, fixed storage policy needs pruning"));
  }
  {
    using StaticWaitSet = rclcpp::StaticWaitSet<0, 0, 0, 1, 0, 0>;
    RCLCPP_EXPECT_THROW_EQ(
      std::make_shared<StaticWaitSet>().reset(),
      std::runtime_error("unexpected condition, fixed storage policy needs pruning"));
  }
  {
    using StaticWaitSet = rclcpp::StaticWaitSet<0, 0, 0, 0, 1, 0>;
    RCLCPP_EXPECT_THROW_EQ(
      std::make_shared<StaticWaitSet>().reset(),
      std::runtime_error("unexpected condition, fixed storage policy needs pruning"));
  }
  {
    using StaticWaitSet = rclcpp::StaticWaitSet<0, 0, 0, 0, 0, 1>;
    RCLCPP_EXPECT_THROW_EQ(
      std::make_shared<StaticWaitSet>().reset(),
      std::runtime_error("unexpected condition, fixed storage policy needs pruning"));
  }
}

TEST_F(TestStaticStorage, wait_subscription) {
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("topic", 10);
  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    "topic", 10, [](test_msgs::msg::Empty::SharedPtr) {});
  rclcpp::SubscriptionWaitSetMask mask{true, true, true};
  rclcpp::StaticWaitSet<1, 0, 0, 0, 0, 0> wait_set({{{subscription, mask}}});

  {
    auto wait_result = wait_set.wait(std::chrono::milliseconds(10));
    EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_result.kind());
  }

  publisher->publish(test_msgs::msg::Empty());
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(-1));
    EXPECT_EQ(rclcpp::WaitResultKind::Ready, wait_result.kind());
  }
}

TEST_F(TestStaticStorage, wait_timer) {
  auto timer = node->create_wall_timer(std::chrono::milliseconds(1), []() {});
  rclcpp::StaticWaitSet<0, 0, 1, 0, 0, 0> wait_set({}, {}, {timer});
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(-1));
    EXPECT_EQ(rclcpp::WaitResultKind::Ready, wait_result.kind());
  }
}

TEST_F(TestStaticStorage, wait_guard_condition) {
  auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
  rclcpp::StaticWaitSet<0, 1, 0, 0, 0, 0> wait_set({}, {guard_condition});

  {
    auto wait_result = wait_set.wait(std::chrono::milliseconds(10));
    EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_result.kind());
  }

  guard_condition->trigger();
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(-1));
    EXPECT_EQ(rclcpp::WaitResultKind::Ready, wait_result.kind());
  }
}

TEST_F(TestStaticStorage, wait_client_service) {
  auto service =
    node->create_service<test_msgs::srv::Empty>(
    "service",
    [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {});

  auto client = node->create_client<test_msgs::srv::Empty>("service");
  rclcpp::StaticWaitSet<0, 0, 0, 1, 1, 0> wait_set({}, {}, {}, {client}, {service});
  {
    auto wait_result = wait_set.wait(std::chrono::milliseconds(10));
    EXPECT_EQ(rclcpp::WaitResultKind::Timeout, wait_result.kind());
  }
  client->async_send_request(std::make_shared<test_msgs::srv::Empty::Request>());
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(-1));
    EXPECT_EQ(rclcpp::WaitResultKind::Ready, wait_result.kind());
  }
}

TEST_F(TestStaticStorage, wait_waitable) {
  auto waitable = std::make_shared<TestWaitable>();
  rclcpp::StaticWaitSet<0, 0, 0, 0, 0, 1> wait_set({}, {}, {}, {}, {}, {{{waitable}}});
  {
    // This waitable doesn't add itself to the rcl_wait_set_t, so Empty is to be expected
    auto wait_result = wait_set.wait(std::chrono::seconds(-1));
    EXPECT_EQ(rclcpp::WaitResultKind::Empty, wait_result.kind());
  }
}
