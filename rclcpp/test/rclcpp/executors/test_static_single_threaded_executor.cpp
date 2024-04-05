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

#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "test_msgs/srv/empty.hpp"

#include "../../mocking_utils/patch.hpp"
#include "../../utils/rclcpp_gtest_macros.hpp"

using namespace std::chrono_literals;

class TestStaticSingleThreadedExecutor : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestStaticSingleThreadedExecutor, add_callback_group_trigger_guard_failed) {
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      executor.add_callback_group(cb_group, node->get_node_base_interface(), true),
      std::runtime_error("Failed to trigger guard condition on callback group add: error not set"));
  }
}

TEST_F(TestStaticSingleThreadedExecutor, add_node_trigger_guard_failed) {
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      executor.add_node(node),
      std::runtime_error("Failed to trigger guard condition on node add: error not set"));
  }
}

TEST_F(TestStaticSingleThreadedExecutor, remove_callback_group_trigger_guard_failed) {
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  executor.add_callback_group(cb_group, node->get_node_base_interface(), true);

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      executor.remove_callback_group(cb_group, true),
      std::runtime_error(
        "Failed to trigger guard condition on callback group remove: error not set"));
  }
}

TEST_F(TestStaticSingleThreadedExecutor, remove_node_failed) {
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      executor.remove_node(node, true),
      std::runtime_error("Node '/ns/node' needs to be associated with an executor."));
  }
}

TEST_F(TestStaticSingleThreadedExecutor, remove_node_trigger_guard_failed) {
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");

  executor.add_node(node);

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
    RCLCPP_EXPECT_THROW_EQ(
      executor.remove_node(node, true),
      std::runtime_error("Failed to trigger guard condition on node remove: error not set"));
  }
}

TEST_F(TestStaticSingleThreadedExecutor, execute_service) {
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  executor.add_node(node);

  auto service =
    node->create_service<test_msgs::srv::Empty>(
    "service",
    [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {});
  auto client = node->create_client<test_msgs::srv::Empty>("service");

  std::promise<void> promise;
  std::future<void> future = promise.get_future();
  EXPECT_EQ(
    rclcpp::FutureReturnCode::TIMEOUT,
    executor.spin_until_future_complete(future, std::chrono::milliseconds(1)));

  executor.remove_node(node);
  executor.spin_until_future_complete(future, std::chrono::milliseconds(1));
}
