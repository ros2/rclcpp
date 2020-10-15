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
#include <utility>
#include <vector>

#include "rclcpp/node.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/msg/empty.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"

// Note: This is a long running test with rmw_connext_cpp, if you change this file, please check
// that this test can complete fully, or adjust the timeout as necessary.
// See https://github.com/ros2/rmw_connext/issues/325 for resolution]

using namespace std::chrono_literals;

template<typename T>
class TestAddCallbackGroupsToExecutor : public ::testing::Test
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
};

using ExecutorTypes =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::executors::StaticSingleThreadedExecutor>;

class ExecutorTypeNames
{
public:
  template<typename T>
  static std::string GetName(int idx)
  {
    (void)idx;
    if (std::is_same<T, rclcpp::executors::SingleThreadedExecutor>()) {
      return "SingleThreadedExecutor";
    }

    if (std::is_same<T, rclcpp::executors::MultiThreadedExecutor>()) {
      return "MultiThreadedExecutor";
    }

    if (std::is_same<T, rclcpp::executors::StaticSingleThreadedExecutor>()) {
      return "StaticSingleThreadedExecutor";
    }

    return "";
  }
};

TYPED_TEST_SUITE(TestAddCallbackGroupsToExecutor, ExecutorTypes, ExecutorTypeNames);

/*
 * Test adding callback groups.
 */
TYPED_TEST(TestAddCallbackGroupsToExecutor, add_callback_groups) {
  using ExecutorType = TypeParam;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  ExecutorType executor;
  executor.add_callback_group(cb_grp, node->get_node_base_interface());
  ASSERT_EQ(executor.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(executor.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(executor.get_automatically_added_callback_groups_from_nodes().size(), 0u);

  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  rclcpp::CallbackGroup::SharedPtr cb_grp2 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cb_grp2;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback, options);
  executor.add_callback_group(cb_grp2, node->get_node_base_interface());
  ASSERT_EQ(executor.get_all_callback_groups().size(), 2u);
  ASSERT_EQ(executor.get_manually_added_callback_groups().size(), 2u);
  ASSERT_EQ(executor.get_automatically_added_callback_groups_from_nodes().size(), 0u);

  executor.add_node(node);
  ASSERT_EQ(executor.get_manually_added_callback_groups().size(), 2u);
  ASSERT_EQ(executor.get_automatically_added_callback_groups_from_nodes().size(), 1u);

  executor.remove_node(node);
  ASSERT_EQ(executor.get_manually_added_callback_groups().size(), 2u);
  ASSERT_EQ(executor.get_automatically_added_callback_groups_from_nodes().size(), 0u);

  executor.remove_callback_group(cb_grp);
  ASSERT_EQ(executor.get_manually_added_callback_groups().size(), 1u);
  ASSERT_EQ(executor.get_automatically_added_callback_groups_from_nodes().size(), 0u);

  executor.remove_callback_group(cb_grp2);
  ASSERT_EQ(executor.get_manually_added_callback_groups().size(), 0u);
  ASSERT_EQ(executor.get_automatically_added_callback_groups_from_nodes().size(), 0u);
}

/*
 * Test removing callback groups.
 */
TYPED_TEST(TestAddCallbackGroupsToExecutor, remove_callback_groups) {
  using ExecutorType = TypeParam;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  ExecutorType executor;
  executor.add_callback_group(cb_grp, node->get_node_base_interface());
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  rclcpp::CallbackGroup::SharedPtr cb_grp2 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = cb_grp2;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback, options);
  executor.add_callback_group(cb_grp2, node->get_node_base_interface());

  executor.remove_callback_group(cb_grp);
  ASSERT_EQ(executor.get_all_callback_groups().size(), 1u);
  executor.remove_callback_group(cb_grp2);
  ASSERT_EQ(executor.get_all_callback_groups().size(), 0u);
}

/*
 * Test adding duplicate callback groups to executor.
 */
TYPED_TEST(TestAddCallbackGroupsToExecutor, add_duplicate_callback_groups)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
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
TYPED_TEST(TestAddCallbackGroupsToExecutor, add_callback_groups_after_add_node_to_executor)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  executor.add_node(node->get_node_base_interface());
  ASSERT_EQ(executor.get_all_callback_groups().size(), 1u);
  std::atomic_int timer_count {0};
  auto timer_callback = [&executor, &timer_count]() {
      if (timer_count > 0) {
        ASSERT_EQ(executor.get_all_callback_groups().size(), 3u);
        executor.cancel();
      }
      timer_count++;
    };
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  rclcpp::CallbackGroup::SharedPtr cb_grp2 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto timer2_callback = []() {};
  rclcpp::TimerBase::SharedPtr timer2_ = node->create_wall_timer(
    2s, timer2_callback, cb_grp2);
  rclcpp::CallbackGroup::SharedPtr cb_grp3 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, true);
  auto timer3_callback = []() {};
  rclcpp::TimerBase::SharedPtr timer3_ = node->create_wall_timer(
    2s, timer3_callback, cb_grp3);
  executor.spin();
}

/*
 * Test adding unallowable callback group.
 */
TYPED_TEST(TestAddCallbackGroupsToExecutor, add_unallowable_callback_groups)
{
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_callback_group(cb_grp, node->get_node_base_interface());
  ASSERT_EQ(executor.get_all_callback_groups().size(), 1u);

  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  rclcpp::CallbackGroup::SharedPtr cb_grp2 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  options.callback_group = cb_grp2;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback, options);
  executor.add_callback_group(cb_grp2, node->get_node_base_interface());
  ASSERT_EQ(executor.get_all_callback_groups().size(), 2u);

  auto timer2_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp3 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::TimerBase::SharedPtr timer2_ = node->create_wall_timer(
    2s, timer2_callback, cb_grp3);
  executor.add_node(node->get_node_base_interface());
  ASSERT_EQ(executor.get_all_callback_groups().size(), 3u);
}

/*
 * Test callback groups from one node to many executors.
 */
TYPED_TEST(TestAddCallbackGroupsToExecutor, one_node_many_callback_groups_many_executors)
{
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  rclcpp::executors::MultiThreadedExecutor timer_executor;
  rclcpp::executors::MultiThreadedExecutor sub_executor;
  timer_executor.add_callback_group(cb_grp, node->get_node_base_interface());
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](const test_msgs::msg::Empty::SharedPtr) {};
  rclcpp::CallbackGroup::SharedPtr cb_grp2 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  options.callback_group = cb_grp2;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, callback, options);
  sub_executor.add_callback_group(cb_grp2, node->get_node_base_interface());
  ASSERT_EQ(sub_executor.get_all_callback_groups().size(), 1u);
  ASSERT_EQ(timer_executor.get_all_callback_groups().size(), 1u);
  auto timer2_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp3 = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::TimerBase::SharedPtr timer2 = node->create_wall_timer(
    2s, timer2_callback, cb_grp3);
  sub_executor.add_node(node);
  ASSERT_EQ(sub_executor.get_all_callback_groups().size(), 2u);
  timer_executor.add_callback_group(cb_grp3, node->get_node_base_interface());
  ASSERT_EQ(timer_executor.get_all_callback_groups().size(), 2u);
}

/*
 * Test removing callback group from executor that its not associated with.
 */
TYPED_TEST(TestAddCallbackGroupsToExecutor, remove_callback_group)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  EXPECT_THROW(
    executor.remove_callback_group(cb_grp),
    std::exception);
  executor.add_callback_group(cb_grp, node->get_node_base_interface());
  EXPECT_NO_THROW(executor.remove_callback_group(cb_grp));
  EXPECT_THROW(
    executor.remove_callback_group(cb_grp),
    std::exception);
}
