// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <string>
#include <memory>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

class MyPublisher : public rclcpp::PublisherBase
{
public:
  size_t get_intraprocess_subscription_count()
  {
    if (get_intra_process_subscription_count_) {
      return get_intra_process_subscription_count_(intra_process_publisher_id_);
    }
    return 0;
  }
};

using rcl_interfaces::msg::IntraProcessMessage;

class TestPublisherSubscriberCount : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>(
      "my_node",
      "/ns",
      rclcpp::NodeOptions().use_intra_process_comms(true));
    publisher = node->create_publisher<IntraProcessMessage>("/topic");
  }

  void TearDown()
  {
    node.reset();
  }

  void test_common(rclcpp::NodeOptions options, const uint64_t intraprocess_count_results[2]);

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::PublisherBase> publisher;
  static std::chrono::milliseconds offset;
};

std::chrono::milliseconds TestPublisherSubscriberCount::offset = std::chrono::milliseconds(500);

void OnMessage(const rcl_interfaces::msg::IntraProcessMessage::SharedPtr msg)
{
  (void)msg;
}

void TestPublisherSubscriberCount::test_common(
  rclcpp::NodeOptions options,
  const uint64_t intraprocess_count_results[2])
{
  EXPECT_EQ(publisher->get_subscription_count(), 0u);
  EXPECT_EQ(
    reinterpret_cast<MyPublisher *>(publisher.get())->get_intraprocess_subscription_count(), 0u);
  {
    auto sub = node->create_subscription<IntraProcessMessage>("/topic", &OnMessage);
    rclcpp::sleep_for(offset);
    EXPECT_EQ(publisher->get_subscription_count(), 1u);
    EXPECT_EQ(
      reinterpret_cast<MyPublisher *>(publisher.get())->get_intraprocess_subscription_count(),
      intraprocess_count_results[0]);
    {
      rclcpp::Node::SharedPtr another_node = std::make_shared<rclcpp::Node>(
        "another_node",
        "/ns",
        options);
      auto another_sub =
        another_node->create_subscription<IntraProcessMessage>("/topic", &OnMessage);

      rclcpp::sleep_for(offset);
      EXPECT_EQ(publisher->get_subscription_count(), 2u);
      EXPECT_EQ(
        reinterpret_cast<MyPublisher *>(publisher.get())->get_intraprocess_subscription_count(),
        intraprocess_count_results[1]);
    }
  }
  /**
    * Counts should be zero here, as all are subscriptions are out of scope.
    * Subscriptions count checking is always preceeded with an sleep, as random failures had been
    * detected without it. */
  rclcpp::sleep_for(offset);
  EXPECT_EQ(publisher->get_subscription_count(), 0u);
  EXPECT_EQ(
    reinterpret_cast<MyPublisher *>(publisher.get())->get_intraprocess_subscription_count(),
    0u);
}

/*
   Testing publisher subscriber count api and internal process subscribers.
   Two subscribers in same using intra_process comm.
 */
TEST_F(TestPublisherSubscriberCount, test_two_intra_comm) {
  const uint64_t results[2] = {1u, 2u};
  test_common(rclcpp::NodeOptions().use_intra_process_comms(true), results);
}

/*
   Testing publisher subscriber count api and internal process subscribers.
   Two subscribers, one using intra_process comm and the other not using it.
 */
TEST_F(TestPublisherSubscriberCount, test_one_intra_comm_one_inter_comm) {
  const uint64_t results[2] = {1u, 1u};
  test_common(rclcpp::NodeOptions().use_intra_process_comms(false), results);
}

/*
   Testing publisher subscriber count api and internal process subscribers, with two contexts.
 */
TEST_F(TestPublisherSubscriberCount, test_with_two_contexts_both_using_intra) {
  const uint64_t results[2] = {1u, 1u};
  auto context = rclcpp::Context::make_shared();
  context->init(0, nullptr);
  test_common(
    rclcpp::NodeOptions().context(context).use_intra_process_comms(true),
    results);
}

TEST_F(TestPublisherSubscriberCount, test_with_two_contexts_one_using_intra) {
  const uint64_t results[2] = {1u, 1u};
  auto context = rclcpp::Context::make_shared();
  context->init(0, nullptr);
  test_common(
    rclcpp::NodeOptions().context(context).use_intra_process_comms(false),
    results);
}
