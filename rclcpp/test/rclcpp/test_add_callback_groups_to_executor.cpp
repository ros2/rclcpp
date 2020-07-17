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
#include <map>
#include <memory>
#include <string>

#include "rclcpp/node.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/empty.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TestAddCallbackGroupsToExecutor : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

/*
 * Test adding callback groups.
 */
TEST_F(TestAddCallbackGroupsToExecutor, add_callback_groups) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_callback_group(cb_grp, node->get_node_base_interface());
  ASSERT_EQ(executor.get_callback_groups().size(), 1u);

  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp2 = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cb_grp2;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback, options);
  executor.add_callback_group(cb_grp2, node->get_node_base_interface());
  ASSERT_EQ(executor.get_callback_groups().size(), 2u);
}

/*
 * Test removing callback groups.
 */
TEST_F(TestAddCallbackGroupsToExecutor, remove_callback_groups) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_callback_group(cb_grp, node->get_node_base_interface());
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp2 = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cb_grp2;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback, options);
  executor.add_callback_group(cb_grp2, node->get_node_base_interface());

  executor.remove_callback_group(cb_grp);
  ASSERT_EQ(executor.get_callback_groups().size(), 1u);
  executor.remove_callback_group(cb_grp2);
  ASSERT_EQ(executor.get_callback_groups().size(), 0u);
}

/*
 * Test adding vector of callback groups.
 */
TEST_F(TestAddCallbackGroupsToExecutor, add_vector_of_callback_groups) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  rclcpp::executors::MultiThreadedExecutor executor;
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp2 = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cb_grp2;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback, options);
  std::vector<rclcpp::callback_group::CallbackGroup::SharedPtr> cb_grps;
  cb_grps.push_back(cb_grp);
  cb_grps.push_back(cb_grp2);
  executor.add_callback_groups(cb_grps, node->get_node_base_interface());
  ASSERT_EQ(executor.get_callback_groups().size(), 2u);
}

/*
 * Test adding map of callback groups and different nodes.
 */
TEST_F(TestAddCallbackGroupsToExecutor, add_map_of_callback_groups) {
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  rclcpp::executors::MultiThreadedExecutor executor;
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  auto node2 = std::make_shared<rclcpp::Node>("my_node2", "/ns");
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp2 = node2->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cb_grp2;
  auto subscription =
    node2->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback, options);
  std::vector<rclcpp::callback_group::CallbackGroup::SharedPtr> cb_grps;
  std::vector<rclcpp::callback_group::CallbackGroup::SharedPtr> cb_grps2;
  cb_grps.push_back(cb_grp);
  cb_grps2.push_back(cb_grp2);
  std::map<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    std::vector<rclcpp::CallbackGroup::SharedPtr>> node_to_groups;
  node_to_groups.insert(
    std::pair<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    std::vector<rclcpp::CallbackGroup::SharedPtr>>(node->get_node_base_interface(), cb_grps));
  node_to_groups.insert(
    std::pair<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr,
    std::vector<rclcpp::CallbackGroup::SharedPtr>>(node2->get_node_base_interface(), cb_grps2));
  executor.add_callback_groups(node_to_groups);

  ASSERT_EQ(executor.get_callback_groups().size(), 2u);
}

/*
 * Test adding duplicate callback groups to executor.
 */
TEST_F(TestAddCallbackGroupsToExecutor, add_duplicate_callback_groups)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  executor.add_callback_group(cb_grp, node->get_node_base_interface());
  EXPECT_THROW(
    executor.add_callback_group(cb_grp, node->get_node_base_interface()),
    std::exception);
}

/*
 * Test adding callback group after node is added to executor.
 */
TEST_F(TestAddCallbackGroupsToExecutor, add_callback_groups_after_add_node_to_executor)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  executor.add_node(node->get_node_base_interface());
  ASSERT_EQ(executor.get_callback_groups().size(), 1u);
  std::atomic_int timer_count {0};
  auto timer_callback = [&executor, &timer_count]() {
      if (timer_count > 0) {
        ASSERT_EQ(executor.get_callback_groups().size(), 2u);
        executor.cancel();
      }
      timer_count++;
    };
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  executor.spin();
}
