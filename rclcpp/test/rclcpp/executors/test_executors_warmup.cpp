// Copyright 2024 Open Source Robotics Foundation, Inc.
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

/**
 * This test checks all implementations of rclcpp::executor to check they pass they basic API
 * tests. Anything specific to any executor in particular should go in a separate test file.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/empty.hpp"

#include "./executor_types.hpp"

using namespace std::chrono_literals;

template<typename T>
class TestExecutorsWarmup : public ::testing::Test
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

TYPED_TEST_SUITE(TestExecutorsWarmup, ExecutorTypes, ExecutorTypeNames);

// This test verifies that spin_all is correctly collecting work multiple times
// even when one of the items of work is a notifier waitable event and thus results in
// rebuilding the entities collection.
// When spin_all goes back to collect more work, it should see the ready items from
// the new added entities
TYPED_TEST(TestExecutorsWarmup, spin_all_doesnt_require_warmup)
{
  using ExecutorType = TypeParam;
  ExecutorType executor;

  // Enable intra-process to guarantee deterministic and synchronous delivery of the message / event
  auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("test_node", node_options);

  // Add node to the executor before creating the entities
  executor.add_node(node);

  // Create entities, this will produce a notifier waitable event, telling the executor to refresh
  // the entities collection
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("test_topic", rclcpp::QoS(10));
  size_t callback_count = 0;
  auto callback = [&callback_count](test_msgs::msg::Empty::ConstSharedPtr) {callback_count++;};
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>(
    "test_topic", rclcpp::QoS(10), std::move(callback));

  ASSERT_EQ(callback_count, 0u);

  // Publish a message so that the new entities (i.e. the subscriber) already have work to do
  publisher->publish(test_msgs::msg::Empty());

  // We need to select a duration that is greater than
  // the time taken to refresh the entities collection and rebuild the waitset.
  // spin-all is expected to process the notifier waitable event, rebuild the collection,
  // and then collect more work, finding the subscription message event.
  // This duration has been selected empirically.
  executor.spin_all(std::chrono::milliseconds(500));

  // Verify that the callback is called as part of the spin above
  EXPECT_EQ(callback_count, 1u);
}

// Same test as `spin_all_doesnt_require_warmup`, but uses a callback group
// This test reproduces the bug reported by https://github.com/ros2/rclcpp/issues/2589
TYPED_TEST(TestExecutorsWarmup, spin_all_doesnt_require_warmup_with_cbgroup)
{
  using ExecutorType = TypeParam;

  ExecutorType executor;

  // Enable intra-process to guarantee deterministic and synchronous delivery of the message / event
  auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("test_node", node_options);

  auto callback_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  // Add callback group to the executor before creating the entities
  executor.add_callback_group(callback_group, node->get_node_base_interface());

  // Create entities, this will produce a notifier waitable event, telling the executor to refresh
  // the entities collection
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("test_topic", rclcpp::QoS(10));
  size_t callback_count = 0;
  auto callback = [&callback_count](test_msgs::msg::Empty::ConstSharedPtr) {callback_count++;};
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>(
    "test_topic", rclcpp::QoS(10), std::move(callback), sub_options);

  ASSERT_EQ(callback_count, 0u);

  // Publish a message so that the new entities (i.e. the subscriber) already have work to do
  publisher->publish(test_msgs::msg::Empty());

  // We need to select a duration that is greater than
  // the time taken to refresh the entities collection and rebuild the waitset.
  // spin-all is expected to process the notifier waitable event, rebuild the collection,
  // and then collect more work, finding the subscription message event.
  // This duration has been selected empirically.
  executor.spin_all(std::chrono::milliseconds(500));

  // Verify that the callback is called as part of the spin above
  EXPECT_EQ(callback_count, 1u);
}

TYPED_TEST(TestExecutorsWarmup, spin_some_doesnt_require_warmup)
{
  using ExecutorType = TypeParam;

  // TODO(alsora): currently only the events-executor passes this test.
  // Enable single-threaded and multi-threaded executors
  // when https://github.com/ros2/rclcpp/pull/2595 gets merged
  if (
    !std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>())
  {
    GTEST_SKIP();
  }

  ExecutorType executor;

  // Enable intra-process to guarantee deterministic and synchronous delivery of the message / event
  auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("test_node", node_options);

  // Add node to the executor before creating the entities
  executor.add_node(node);

  // Create entities, this will produce a notifier waitable event, telling the executor to refresh
  // the entities collection
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("test_topic", rclcpp::QoS(10));
  size_t callback_count = 0;
  auto callback = [&callback_count](test_msgs::msg::Empty::ConstSharedPtr) {callback_count++;};
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>(
    "test_topic", rclcpp::QoS(10), std::move(callback));

  ASSERT_EQ(callback_count, 0u);

  // Publish a message so that the new entities (i.e. the subscriber) already have work to do
  publisher->publish(test_msgs::msg::Empty());

  // NOTE: intra-process communication is enabled, so the subscription will immediately see
  // the new message, no risk of race conditions where spin_some gets called before the
  // message has been delivered.
  executor.spin_some();

  // Verify that the callback is called as part of the spin above
  EXPECT_EQ(callback_count, 1u);
}

TYPED_TEST(TestExecutorsWarmup, spin_some_doesnt_require_warmup_with_cbgroup)
{
  using ExecutorType = TypeParam;

  // TODO(alsora): currently only the events-executor passes this test.
  // Enable single-threaded and multi-threaded executors
  // when https://github.com/ros2/rclcpp/pull/2595 gets merged
  if (
    !std::is_same<ExecutorType, rclcpp::experimental::executors::EventsExecutor>())
  {
    GTEST_SKIP();
  }

  ExecutorType executor;

  // Enable intra-process to guarantee deterministic and synchronous delivery of the message / event
  auto node_options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<rclcpp::Node>("test_node", node_options);

  auto callback_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

  // Add callback group to the executor before creating the entities
  executor.add_callback_group(callback_group, node->get_node_base_interface());

  // Create entities, this will produce a notifier waitable event, telling the executor to refresh
  // the entities collection
  auto publisher = node->create_publisher<test_msgs::msg::Empty>("test_topic", rclcpp::QoS(10));
  size_t callback_count = 0;
  auto callback = [&callback_count](test_msgs::msg::Empty::ConstSharedPtr) {callback_count++;};
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group;
  auto subscription =
    node->create_subscription<test_msgs::msg::Empty>(
    "test_topic", rclcpp::QoS(10), std::move(callback), sub_options);

  ASSERT_EQ(callback_count, 0u);

  // Publish a message so that the new entities (i.e. the subscriber) already have work to do
  publisher->publish(test_msgs::msg::Empty());

  // NOTE: intra-process communication is enabled, so the subscription will immediately see
  // the new message, no risk of race conditions where spin_some gets called before the
  // message has been delivered.
  executor.spin_some();

  // Verify that the callback is called as part of the spin above
  EXPECT_EQ(callback_count, 1u);
}
