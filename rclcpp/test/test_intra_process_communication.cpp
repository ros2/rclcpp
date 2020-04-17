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

#include <test_msgs/message_fixtures.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/experimental/serialized_message.hpp"

int32_t & get_test_allocation_counter()
{
  static int32_t counter = 0;
  return counter;
}

void * custom_allocate(size_t size, void * state)
{
  static auto m_allocator = rcutils_get_default_allocator();

  ++get_test_allocation_counter();
  auto r = m_allocator.allocate(size, state);
  return r;
}

void * custom_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
{
  static auto m_allocator = rcutils_get_default_allocator();

  ++get_test_allocation_counter();
  auto r = m_allocator.zero_allocate(number_of_elements, size_of_element, state);
  return r;
}

void * custom_reallocate(void * pointer, size_t size, void * state)
{
  static auto m_allocator = rcutils_get_default_allocator();

  if (pointer == nullptr) {
    ++get_test_allocation_counter();
  }

  auto r = m_allocator.reallocate(pointer, size, state);
  return r;
}

void custom_deallocate(void * pointer, void * state)
{
  static auto m_allocator = rcutils_get_default_allocator();

  --get_test_allocation_counter();
  m_allocator.deallocate(pointer, state);
}

rcl_serialized_message_t make_serialized_string_msg(
  const std::shared_ptr<test_msgs::msg::Strings> & stringMsg)
{
  auto m_allocator = rcutils_get_default_allocator();

  // add custom (de)allocator to count the references to the object
  m_allocator.allocate = &custom_allocate;
  m_allocator.deallocate = &custom_deallocate;
  m_allocator.reallocate = &custom_reallocate;
  m_allocator.zero_allocate = &custom_zero_allocate;

  rcl_serialized_message_t msg = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&msg, 0, &m_allocator);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }

  static auto type =
    rosidl_typesupport_cpp::get_message_type_support_handle
    <test_msgs::msg::Strings>();
  auto error = rmw_serialize(stringMsg.get(), type, &msg);
  if (error != RMW_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "test_intra_process_communication",
      "Something went wrong preparing the serialized message");
  }

  return msg;
}

/**
 * Parameterized test.
 * The first param are the NodeOptions used to create the nodes.
 * The second param are the expected intraprocess count results.
 */
struct TestParameters
{
  rclcpp::NodeOptions node_options[2];
  uint64_t intraprocess_count_results[2];
  size_t runs;
  std::string description;
};

std::ostream & operator<<(std::ostream & out, const TestParameters & params)
{
  out << params.description;
  return out;
}

class TestPublisherSubscriptionSerialized : public ::testing::TestWithParam<TestParameters>
{
public:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

protected:
  static std::chrono::milliseconds offset;
};

std::chrono::milliseconds TestPublisherSubscriptionSerialized::offset = std::chrono::milliseconds(
  250);
std::array<uint32_t, 2> counts;

void OnMessageSerialized(const std::shared_ptr<const rcl_serialized_message_t> msg)
{
  EXPECT_NE(msg->buffer, nullptr);
  EXPECT_GT(msg->buffer_capacity, 0u);

  ++counts[0];
}

void OnMessageConst(std::shared_ptr<const test_msgs::msg::Strings> msg)
{
  EXPECT_EQ(msg->string_value.back(), '9');

  ++counts[1];
}

void OnMessageUniquePtr(std::unique_ptr<test_msgs::msg::Strings> msg)
{
  EXPECT_EQ(msg->string_value.back(), '9');

  ++counts[1];
}

void OnMessage(std::shared_ptr<test_msgs::msg::Strings> msg)
{
  EXPECT_EQ(msg->string_value.back(), '9');

  ++counts[1];
}

TEST_P(TestPublisherSubscriptionSerialized, publish_serialized)
{
  get_test_allocation_counter() = 0;

  TestParameters parameters = GetParam();
  {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
      "my_node",
      "/ns",
      parameters.node_options[0]);
    auto publisher = node->create_publisher<test_msgs::msg::Strings>("/topic", 10);

    auto sub_shared = node->create_subscription<test_msgs::msg::Strings>(
      "/topic", 10,
      &OnMessage);
    auto sub_unique = node->create_subscription<test_msgs::msg::Strings>(
      "/topic", 10,
      &OnMessageUniquePtr);
    auto sub_const_shared = node->create_subscription<test_msgs::msg::Strings>(
      "/topic", 10,
      &OnMessageConst);
    auto sub_serialized = node->create_subscription<test_msgs::msg::Strings>(
      "/topic", 10,
      &OnMessageSerialized);

    rclcpp::sleep_for(offset);

    counts.fill(0);
    auto stringMsg = get_messages_strings()[3];

    for (size_t i = 0; i < parameters.runs; i++) {
      auto msg0 = make_serialized_string_msg(stringMsg);

      std::unique_ptr<test_msgs::msg::Strings> stringMsgU(
        new test_msgs::msg::Strings(
          *stringMsg));

      publisher->publish(std::make_unique<rcl_serialized_message_t>(msg0));
      publisher->publish(*stringMsg);
      publisher->publish(std::move(stringMsgU));
    }
    for (uint32_t i = 0; i < 3; ++i) {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(offset);
    }

    rclcpp::spin_some(node);
  }

  if (parameters.runs == 1) {
    EXPECT_EQ(counts[0], 3u);
    EXPECT_EQ(counts[1], 9u);
  }

  EXPECT_EQ(get_test_allocation_counter(), 0);
}

TEST_P(TestPublisherSubscriptionSerialized, publish_serialized_generic)
{
  get_test_allocation_counter() = 0;

  TestParameters parameters = GetParam();
  {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
      "my_node",
      "/ns",
      parameters.node_options[0]);
    auto publisher = rclcpp::create_publisher<rcl_serialized_message_t>(
      node,
      "/topic",
      *rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Strings>(),
      rclcpp::QoS(10));

    auto sub_gen_serialized = rclcpp::create_subscription<rcl_serialized_message_t>(
      node,
      "/topic",
      *rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Strings>(),
      rclcpp::QoS(10),
      &OnMessageSerialized);

    auto sub_shared = node->create_subscription<test_msgs::msg::Strings>(
      "/topic", 10,
      &OnMessage);
    auto sub_unique = node->create_subscription<test_msgs::msg::Strings>(
      "/topic", 10,
      &OnMessageUniquePtr);
    auto sub_const_shared = node->create_subscription<test_msgs::msg::Strings>(
      "/topic", 10,
      &OnMessageConst);
    auto sub_serialized = node->create_subscription<test_msgs::msg::Strings>(
      "/topic", 10,
      &OnMessageSerialized);

    rclcpp::sleep_for(offset);

    counts.fill(0);
    auto stringMsg = get_messages_strings()[3];

    for (size_t i = 0; i < parameters.runs; i++) {
      auto msg0 = make_serialized_string_msg(stringMsg);

      publisher->publish(std::make_unique<rcl_serialized_message_t>(msg0));
    }
    rclcpp::spin_some(node);
    rclcpp::sleep_for(offset);

    rclcpp::spin_some(node);
  }

  if (parameters.runs == 1) {
    EXPECT_EQ(counts[0], 2u);
    EXPECT_EQ(counts[1], 3u);
  }

  EXPECT_EQ(get_test_allocation_counter(), 0);
}

auto get_new_context()
{
  auto context = rclcpp::Context::make_shared();
  context->init(0, nullptr);
  return context;
}

std::vector<TestParameters> parameters = {
  /*
     Testing publisher subscription count api and internal process subscription count.
     Two subscriptions in the same topic, both using intraprocess comm.
   */
  {
    {
      rclcpp::NodeOptions().use_intra_process_comms(true),
      rclcpp::NodeOptions().use_intra_process_comms(true)
    },
    {1u, 2u},
    1,
    "two_subscriptions_intraprocess_comm"
  },
  /*
     Testing publisher subscription count api and internal process subscription count.
     Two subscriptions, one using intra-process comm and the other not using it.
   */
  {
    {
      rclcpp::NodeOptions().use_intra_process_comms(true),
      rclcpp::NodeOptions().use_intra_process_comms(false)
    },
    {1u, 1u},
    1,
    "two_subscriptions_one_intraprocess_one_not"
  },
  /*
     Testing publisher subscription count api and internal process subscription count.
     Two contexts, both using intra-process.
   */
  {
    {
      rclcpp::NodeOptions().use_intra_process_comms(true),
      rclcpp::NodeOptions().context(get_new_context()).use_intra_process_comms(true)
    },
    {1u, 1u},
    1,
    "two_subscriptions_in_two_contexts_with_intraprocess_comm"
  },
  /*
     Testing publisher subscription count api and internal process subscription count.
     Two contexts, both of them not using intra-process comm.
   */
  {
    {
      rclcpp::NodeOptions().use_intra_process_comms(false),
      rclcpp::NodeOptions().context(get_new_context()).use_intra_process_comms(false)
    },
    {0u, 0u},
    1,
    "two_subscriptions_in_two_contexts_without_intraprocess_comm"
  }
};

std::vector<TestParameters> setRuns(const std::vector<TestParameters> & in, const size_t runs)
{
  std::vector<TestParameters> out = in;
  for (auto & p : out) {
    p.runs = runs;
  }
  return out;
}

INSTANTIATE_TEST_CASE_P(
  TestWithDifferentNodeOptions, TestPublisherSubscriptionSerialized,
  ::testing::ValuesIn(parameters),
  ::testing::PrintToStringParamName());

INSTANTIATE_TEST_CASE_P(
  TestWithDifferentNodeOptions1000Runs, TestPublisherSubscriptionSerialized,
  ::testing::ValuesIn(setRuns(parameters, 1000)),
  ::testing::PrintToStringParamName());
