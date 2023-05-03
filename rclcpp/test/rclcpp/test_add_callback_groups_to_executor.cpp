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

template<typename T>
class TestAddCallbackGroupsToExecutorStable : public TestAddCallbackGroupsToExecutor<T> {};

using ExecutorTypes =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::executors::StaticSingleThreadedExecutor,
  rclcpp::experimental::executors::EventsExecutor>;

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

    if (std::is_same<T, rclcpp::experimental::executors::EventsExecutor>()) {
      return "EventsExecutor";
    }

    return "";
  }
};

TYPED_TEST_SUITE(TestAddCallbackGroupsToExecutor, ExecutorTypes, ExecutorTypeNames);

// StaticSingleThreadedExecutor is not included in these tests for now
using StandardExecutors =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::experimental::executors::EventsExecutor>;
TYPED_TEST_SUITE(TestAddCallbackGroupsToExecutorStable, StandardExecutors, ExecutorTypeNames);

/*
 * Test adding callback groups.
 */
TYPED_TEST(TestAddCallbackGroupsToExecutor, add_callback_groups)
{
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

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
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
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
TYPED_TEST(TestAddCallbackGroupsToExecutor, remove_callback_groups)
{
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

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
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
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
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

  ExecutorType executor;
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
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

  ExecutorType executor;
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
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

  ExecutorType executor;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  executor.add_callback_group(cb_grp, node->get_node_base_interface());
  ASSERT_EQ(executor.get_all_callback_groups().size(), 1u);

  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
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
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

  ExecutorType timer_executor;
  ExecutorType sub_executor;
  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto timer_callback = []() {};
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::TimerBase::SharedPtr timer_ = node->create_wall_timer(
    2s, timer_callback, cb_grp);
  timer_executor.add_callback_group(cb_grp, node->get_node_base_interface());
  const rclcpp::QoS qos(10);
  auto options = rclcpp::SubscriptionOptions();
  auto callback = [](test_msgs::msg::Empty::ConstSharedPtr) {};
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
 * Test callback groups from one node to many executors.
 * A subscriber on a new executor with a callback group not received a message
 * because the executor can't be triggered while a subscriber created, see
 * https://github.com/ros2/rclcpp/issues/1611
*/
TYPED_TEST(TestAddCallbackGroupsToExecutorStable, subscriber_triggered_to_receive_message)
{
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

  // create a thread running an executor with a new callback group for a coming subscriber
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  ExecutorType cb_grp_executor;

  std::promise<bool> received_message_promise;
  auto received_message_future = received_message_promise.get_future();
  rclcpp::FutureReturnCode return_code = rclcpp::FutureReturnCode::TIMEOUT;
  std::thread cb_grp_thread = std::thread(
    [&cb_grp, &node, &cb_grp_executor, &received_message_future, &return_code]() {
      cb_grp_executor.add_callback_group(cb_grp, node->get_node_base_interface());
      return_code = cb_grp_executor.spin_until_future_complete(received_message_future, 10s);
    });

  // expect the subscriber to receive a message
  auto sub_callback = [&received_message_promise](test_msgs::msg::Empty::ConstSharedPtr) {
      received_message_promise.set_value(true);
    };

  rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr subscription;
  rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr publisher;
  // to create a timer with a callback run on another executor
  rclcpp::TimerBase::SharedPtr timer = nullptr;
  std::promise<void> timer_promise;
  auto timer_callback =
    [&subscription, &publisher, &timer, &cb_grp, &node, &sub_callback, &timer_promise]() {
      if (timer) {
        timer.reset();
      }

      // create a subscription using the `cb_grp` callback group
      rclcpp::QoS qos = rclcpp::QoS(1).reliable();
      auto options = rclcpp::SubscriptionOptions();
      options.callback_group = cb_grp;
      subscription =
        node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, sub_callback, options);
      // create a publisher to send data
      publisher =
        node->create_publisher<test_msgs::msg::Empty>("topic_name", qos);
      publisher->publish(test_msgs::msg::Empty());
      timer_promise.set_value();
    };

  ExecutorType timer_executor;
  timer = node->create_wall_timer(100ms, timer_callback);
  timer_executor.add_node(node);
  auto future = timer_promise.get_future();
  timer_executor.spin_until_future_complete(future);
  cb_grp_thread.join();

  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
  EXPECT_TRUE(received_message_future.get());
}

/*
 * Test callback group created after spin.
 * A subscriber with a new callback group that created after executor spin not received a message
 * because the executor can't be triggered while a subscriber created, see
 * https://github.com/ros2/rclcpp/issues/2067
*/
TYPED_TEST(TestAddCallbackGroupsToExecutorStable, callback_group_create_after_spin)
{
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

  auto node = std::make_shared<rclcpp::Node>("my_node", "/ns");

  // create a publisher to send data
  rclcpp::QoS qos = rclcpp::QoS(1).reliable().transient_local();
  rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr publisher =
    node->create_publisher<test_msgs::msg::Empty>("topic_name", qos);
  publisher->publish(test_msgs::msg::Empty());

  // create a thread running an executor
  ExecutorType executor;
  executor.add_node(node);
  std::promise<bool> received_message_promise;
  auto received_message_future = received_message_promise.get_future();
  rclcpp::FutureReturnCode return_code = rclcpp::FutureReturnCode::TIMEOUT;
  std::thread executor_thread = std::thread(
    [&executor, &received_message_future, &return_code]() {
      return_code = executor.spin_until_future_complete(received_message_future, 5s);
    });

  // to create a callback group after spin
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  rclcpp::CallbackGroup::SharedPtr cb_grp = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // expect the subscriber to receive a message
  auto sub_callback = [&received_message_promise](test_msgs::msg::Empty::ConstSharedPtr) {
      received_message_promise.set_value(true);
    };
  // create a subscription using the `cb_grp` callback group
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = cb_grp;
  rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr subscription =
    node->create_subscription<test_msgs::msg::Empty>("topic_name", qos, sub_callback, options);

  executor_thread.join();
  ASSERT_EQ(rclcpp::FutureReturnCode::SUCCESS, return_code);
  EXPECT_TRUE(received_message_future.get());
}

/*
 * Test removing callback group from executor that its not associated with.
 */
TYPED_TEST(TestAddCallbackGroupsToExecutor, remove_callback_group)
{
  using ExecutorType = TypeParam;
  // rmw_connextdds doesn't support events-executor
  if (
    std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>() &&
    std::string(rmw_get_implementation_identifier()).find("rmw_connextdds") == 0)
  {
    GTEST_SKIP();
  }

  ExecutorType executor;
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
