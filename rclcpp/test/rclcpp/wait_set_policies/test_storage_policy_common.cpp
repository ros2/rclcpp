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
#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

class TestStoragePolicyCommon : public ::testing::Test
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
  : is_ready_(false), add_to_wait_set_(false) {}
  bool add_to_wait_set(rcl_wait_set_t *) override {return add_to_wait_set_;}

  bool is_ready(rcl_wait_set_t *) override {return is_ready_;}

  void execute() override {}

  void set_is_ready(bool value) {is_ready_ = value;}

  void set_add_to_wait_set(bool value) {add_to_wait_set_ = value;}

private:
  bool is_ready_;
  bool add_to_wait_set_;
};

TEST_F(TestStoragePolicyCommon, rcl_wait_set_fini_error) {
  auto wait_set = std::make_shared<rclcpp::WaitSet>();
  auto mock = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_wait_set_fini, RCL_RET_ERROR);
  EXPECT_NO_THROW(wait_set.reset());
}

TEST_F(TestStoragePolicyCommon, rcl_wait_set_resize_error) {
  rclcpp::WaitSet wait_set;

  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    "topic", 10, [](test_msgs::msg::Empty::SharedPtr) {});
  rclcpp::SubscriptionWaitSetMask mask{true, true, true};

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_resize, RCL_RET_ERROR);
  wait_set.add_subscription(subscription, mask);
  EXPECT_THROW(
    wait_set.wait(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestStoragePolicyCommon, rcl_wait_set_clear_error) {
  rclcpp::WaitSet wait_set;

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_clear, RCL_RET_ERROR);
  EXPECT_THROW(
    wait_set.wait(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestStoragePolicyCommon, rcl_wait_set_add_subscription_error) {
  rclcpp::WaitSet wait_set;
  auto subscription = node->create_subscription<test_msgs::msg::Empty>(
    "topic", 10, [](test_msgs::msg::Empty::SharedPtr) {});
  rclcpp::SubscriptionWaitSetMask mask{true, true, true};

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_add_subscription, RCL_RET_ERROR);
  wait_set.add_subscription(subscription, mask);
  EXPECT_THROW(
    wait_set.wait(),
    rclcpp::exceptions::RCLError);
}
TEST_F(TestStoragePolicyCommon, rcl_wait_set_add_guard_condition_error) {
  rclcpp::WaitSet wait_set;
  auto guard_condition = std::make_shared<rclcpp::GuardCondition>();
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_add_guard_condition, RCL_RET_ERROR);
  wait_set.add_guard_condition(guard_condition);
  EXPECT_THROW(
    wait_set.wait(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestStoragePolicyCommon, rcl_wait_set_add_timer_error) {
  rclcpp::WaitSet wait_set;
  auto timer = node->create_wall_timer(std::chrono::seconds(100), []() {});
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_add_timer, RCL_RET_ERROR);
  wait_set.add_timer(timer);
  EXPECT_THROW(
    wait_set.wait(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestStoragePolicyCommon, rcl_wait_set_add_service_error) {
  rclcpp::WaitSet wait_set;
  auto service =
    node->create_service<test_msgs::srv::Empty>(
    "service",
    [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {});
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_add_service, RCL_RET_ERROR);
  wait_set.add_service(service);
  EXPECT_THROW(
    wait_set.wait(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestStoragePolicyCommon, rcl_wait_set_add_client_error) {
  rclcpp::WaitSet wait_set;
  auto client = node->create_client<test_msgs::srv::Empty>("service");
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_add_client, RCL_RET_ERROR);
  wait_set.add_client(client);
  EXPECT_THROW(
    wait_set.wait(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestStoragePolicyCommon, add_waitable_error) {
  rclcpp::WaitSet wait_set;
  auto waitable = std::make_shared<TestWaitable>();
  waitable->set_add_to_wait_set(false);
  wait_set.add_waitable(waitable);
  RCLCPP_EXPECT_THROW_EQ(
    wait_set.wait(),
    std::runtime_error("waitable unexpectedly failed to be added to wait set"));
}
