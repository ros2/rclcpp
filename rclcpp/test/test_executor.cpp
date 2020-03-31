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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using namespace rclcpp::executors;

template <typename ExecutorT>
class TestExecutors : public ::testing::Test
{
protected:

  using MsgT = test_msgs::msg::Empty;

  class PubNode : public rclcpp::Node
  {
  public:
    PubNode(std::string node_name) : rclcpp::Node(node_name, "", rclcpp::NodeOptions().use_intra_process_comms(true)) { }

    void add_periodic_publisher(){
      _pub = create_publisher<MsgT>("topic", 10);
      auto timer_callback =
        [&]() -> void {
          auto message = MsgT();
          _pub->publish(message);
        };
        _timer = create_wall_timer(25ms, timer_callback);
    }
  private:
    rclcpp::Publisher<MsgT>::SharedPtr _pub;
    rclcpp::TimerBase::SharedPtr _timer;
  };

  class SubNode : public rclcpp::Node
  {
  public:
    SubNode(std::string node_name) : rclcpp::Node(node_name, "", rclcpp::NodeOptions().use_intra_process_comms(true)) { }

    void add_subscription(){
      _sub = create_subscription<MsgT>(
        "topic",
        10,
        [&](MsgT::SharedPtr msg) {
          (void)msg;
          _count++;
        });
    }
    void reset_count() { _count = 0; }
    size_t get_count() { return _count; }
  private:
    rclcpp::Subscription<MsgT>::SharedPtr _sub;
    std::atomic<size_t> _count {0};
  };

  void SetUp()
  {
    rclcpp::init(0, nullptr);
    talker_node = std::make_shared<PubNode>("talker");
    listener_node = std::make_shared<SubNode>("listener");
  }

  void TearDown()
  {
    talker_node.reset();
    listener_node.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<PubNode> talker_node;
  std::shared_ptr<SubNode> listener_node;
};

TYPED_TEST_CASE_P(TestExecutors);

// Make sure that executors detach from nodes when destructing
TYPED_TEST_P(TestExecutors, detachOnDestruction) {
  {
    TypeParam executor;
    executor.add_node(this->talker_node);
  }
  {
    TypeParam executor;
    EXPECT_NO_THROW(executor.add_node(this->talker_node));
  }
}

// Make sure that the executor can automatically remove expired nodes correctly
TYPED_TEST_P(TestExecutors, addTemporaryNode) {
  TypeParam executor;
  executor.add_node(std::make_shared<rclcpp::Node>("temporary_node"));
  EXPECT_NO_THROW(executor.spin_some());
}

// Make sure that the executor can deal with basic pub-sub mechanism
TYPED_TEST_P(TestExecutors, defaultUsage) {

  // Add a publisher in the talker
  this->talker_node->add_periodic_publisher();
  // Add a subscription in the listener
  this->listener_node->add_subscription();

  // Add both nodes to the executor
  auto executor = std::make_shared<TypeParam>();
  executor->add_node(this->talker_node);
  executor->add_node(this->listener_node);

  // Spin for a while
  std::thread executor_thread([=]() { executor->spin(); });
  std::this_thread::sleep_for(100ms);
  executor->cancel();
  executor_thread.join();

  // Remove both nodes from the executor
  executor->remove_node(this->talker_node);
  executor->remove_node(this->listener_node);

  // Make sure that the listener node has reveived messages
  ASSERT_GT(this->listener_node->get_count(), 0u);
}

// Make sure that the executor allows to add new nodes while it's spinning
TYPED_TEST_P(TestExecutors, nodeAddedWhileSpinning) {

  // Add a publisher in the talker
  this->talker_node->add_periodic_publisher();
  // Add a subscription in the listener
  this->listener_node->add_subscription();

  // Add only the talker node to the executor
  auto executor = std::make_shared<TypeParam>();
  executor->add_node(this->talker_node);

  // Start spinning
  std::thread executor_thread([=]() { executor->spin(); });
  std::this_thread::sleep_for(100ms);

  // The listener was not in the executor, so no messages received
  ASSERT_EQ(this->listener_node->get_count(), 0u);

  // Add the listener node to the executor while it's spinning
  executor->add_node(this->listener_node);

  // Keep spinning a little bit more
  std::this_thread::sleep_for(100ms);
  executor->cancel();
  executor_thread.join();

  // Remove both nodes from the executor
  executor->remove_node(this->talker_node);
  executor->remove_node(this->listener_node);

  // Make sure that the listener node has reveived messages
  ASSERT_GT(this->listener_node->get_count(), 0u);
}

// Make sure that the executor allows to add entities to one of its nodes while it's spinning
TYPED_TEST_P(TestExecutors, subAddedWhileSpinning) {

  // Add a publisher in the talker
  this->talker_node->add_periodic_publisher();

  // Add both nodes to the executor
  auto executor = std::make_shared<TypeParam>();
  executor->add_node(this->talker_node);
  executor->add_node(this->listener_node);

  // Start spinning
  std::thread executor_thread([=]() { executor->spin(); });
  std::this_thread::sleep_for(100ms);

  // The listener had no subscriptions, so no messages received
  ASSERT_EQ(this->listener_node->get_count(), 0u);

  // Add a subscription in the listener while the executor is spinning
  this->listener_node->add_subscription();

  // Keep spinning a little bit more
  std::this_thread::sleep_for(100ms);
  executor->cancel();
  executor_thread.join();

  // Remove both nodes from the executor
  executor->remove_node(this->talker_node);
  executor->remove_node(this->listener_node);

  // Make sure that the listener node has reveived messages
  ASSERT_GT(this->listener_node->get_count(), 0u);
}

// Make sure that the executor allows to remove nodes while it's spinning
TYPED_TEST_P(TestExecutors, nodeRemovedWhileSpinning) {

  // Add a publisher in the talker
  this->talker_node->add_periodic_publisher();
  // Add a subscription in the listener
  this->listener_node->add_subscription();

  // Add both node to the executor
  auto executor = std::make_shared<TypeParam>();
  executor->add_node(this->talker_node);
  executor->add_node(this->listener_node);

  // Start spinning
  std::thread executor_thread([=]() { executor->spin(); });
  std::this_thread::sleep_for(100ms);

  // Make sure that the listener node has reveived messages
  ASSERT_GT(this->listener_node->get_count(), 0u);

  // Remove the listener node from the executor while it's spinning
  executor->remove_node(this->listener_node);
  this->listener_node->reset_count();

  // Keep spinning a little bit more
  std::this_thread::sleep_for(100ms);
  executor->cancel();
  executor_thread.join();

  // Make sure that the listener node has reveived no messages after being removed
  ASSERT_EQ(this->listener_node->get_count(), 0u);

  executor->remove_node(this->talker_node);
}

REGISTER_TYPED_TEST_CASE_P(TestExecutors,
    detachOnDestruction,
    addTemporaryNode,
    defaultUsage,
    nodeAddedWhileSpinning,
    subAddedWhileSpinning,
    nodeRemovedWhileSpinning
);

typedef ::testing::Types<SingleThreadedExecutor, StaticSingleThreadedExecutor, MultiThreadedExecutor> ExecutorTypes;
INSTANTIATE_TYPED_TEST_CASE_P(ExecutorTypesInstantiation, TestExecutors, ExecutorTypes);
