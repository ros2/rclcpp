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

#include <rcl_interfaces/msg/intra_process_message.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"


int32_t & get_test_allocation_counter()
{
  static int32_t counter = 0;
  return counter;
}

std::shared_ptr<rcutils_uint8_array_t> make_serialized_string_msg(
  const std::shared_ptr<rcl_interfaces::msg::IntraProcessMessage> & stringMsg)
{
  auto m_allocator = rcutils_get_default_allocator();
  size_t message_size = 80u + static_cast<size_t>(sizeof(rcl_interfaces::msg::IntraProcessMessage));

  auto msg = new rcutils_uint8_array_t;
  *msg = rcutils_get_zero_initialized_uint8_array();
  auto ret = rcutils_uint8_array_init(msg, message_size, &m_allocator);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Error allocating resources " + std::to_string(ret));
  }

  ++get_test_allocation_counter();
  auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
    msg,
    [](rcutils_uint8_array_t * msg) {
      --get_test_allocation_counter();
      int error = rcutils_uint8_array_fini(msg);
      delete msg;
      if (error != RCUTILS_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "m4_test",
          "Leaking memory %i",
          error);
      }
    });

  serialized_data->buffer_length = message_size;

  static auto type =
    rosidl_typesupport_cpp::get_message_type_support_handle
    <rcl_interfaces::msg::IntraProcessMessage>();
  auto error = rmw_serialize(stringMsg.get(), type, serialized_data.get());
  if (error != RMW_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED("m4_test", "Something went wrong preparing the serialized message");
  }

  return serialized_data;
}

/**
 * Parameterized test.
 * The first param are the NodeOptions used to create the nodes.
 * The second param are the expect intraprocess count results.
 */
struct TestParameters
{
  rclcpp::NodeOptions node_options[2];
  uint64_t intraprocess_count_results[2];
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
    rclcpp::init(0, nullptr);
  }

protected:
  void SetUp() {}

  void TearDown() {}

  static std::chrono::milliseconds offset;
};

std::chrono::milliseconds TestPublisherSubscriptionSerialized::offset = std::chrono::milliseconds(
  2000);
std::array<uint32_t, 2> counts;

void OnMessageSerialized(const std::shared_ptr<rmw_serialized_message_t> msg)
{
  EXPECT_NE(msg->buffer, nullptr);
  EXPECT_GT(msg->buffer_capacity, 0u);

  ++counts[0];
}

void OnMessageSerializedU(std::unique_ptr<rmw_serialized_message_t> msg)
{
  EXPECT_NE(msg, nullptr);
  EXPECT_NE(msg->buffer, nullptr);
  EXPECT_GT(msg->buffer_capacity, 0u);

  ++counts[0];
}

void OnMessageConst(std::shared_ptr<const rcl_interfaces::msg::IntraProcessMessage> msg)
{
  EXPECT_EQ(msg->message_sequence, 1234u);

  ++counts[1];
}

void OnMessageU(std::unique_ptr<rcl_interfaces::msg::IntraProcessMessage> msg)
{
  EXPECT_EQ(msg->message_sequence, 1234u);

  ++counts[1];
}

void OnMessage(std::shared_ptr<rcl_interfaces::msg::IntraProcessMessage> msg)
{
  EXPECT_EQ(msg->message_sequence, 1234u);

  ++counts[1];
}

TEST_P(TestPublisherSubscriptionSerialized, publish_serialized)
{
  TestParameters parameters = GetParam();
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
    "my_node",
    "/ns",
    parameters.node_options[0]);
  auto publisher = node->create_publisher<rcl_interfaces::msg::IntraProcessMessage>("/topic", 10);

  auto mem_strategy =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<rcl_interfaces::msg::IntraProcessMessage>
    ::create_default();

  auto stringMsg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  stringMsg->message_sequence = 1234u;

  {
    auto msg0 = make_serialized_string_msg(stringMsg);

    auto sub = node->create_subscription<rcl_interfaces::msg::IntraProcessMessage>("/topic", 10,
        &OnMessage);
    auto sub1 = node->create_subscription<rcl_interfaces::msg::IntraProcessMessage>("/topic", 10,
        &OnMessageU);
    auto sub2 = node->create_subscription<rcl_interfaces::msg::IntraProcessMessage>("/topic", 10,
        &OnMessageConst);
    auto sub_ser = node->create_subscription<rcl_interfaces::msg::IntraProcessMessage>("/topic", 10,
        &OnMessageSerialized);
    auto sub_ser2 = node->create_subscription<rcl_interfaces::msg::IntraProcessMessage>("/topic",
        10,
        &OnMessageSerialized);
    auto sub_ser3 = node->create_subscription<rcl_interfaces::msg::IntraProcessMessage>("/topic",
        10,
        &OnMessageSerializedU);

    rclcpp::sleep_for(offset);

    counts.fill(0);

    std::unique_ptr<rcl_interfaces::msg::IntraProcessMessage> stringMsgU(
      new rcl_interfaces::msg::IntraProcessMessage(
        *stringMsg));
    std::unique_ptr<rcutils_uint8_array_t> msg0U(new rcutils_uint8_array_t(*msg0));

    // Now deprecated functions.
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
    publisher->publish(*msg0);
    publisher->publish(stringMsg);
    publisher->publish(*msg0U);
    publisher->publish(std::move(stringMsgU));
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif

    rclcpp::spin_some(node);
    rclcpp::sleep_for(offset);

    rclcpp::spin_some(node);

    EXPECT_EQ(counts[0], 12u);
    EXPECT_EQ(counts[1], 12u);
  }

  EXPECT_EQ(get_test_allocation_counter(), 0);
}

auto get_new_context()
{
  auto context = rclcpp::Context::make_shared();
  context->init(0, nullptr);
  return context;
}

TestParameters parameters[] = {
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
    "two_subscriptions_in_two_contexts_without_intraprocess_comm"
  }
};

INSTANTIATE_TEST_CASE_P(
  TestWithDifferentNodeOptions, TestPublisherSubscriptionSerialized,
  ::testing::ValuesIn(parameters),
  ::testing::PrintToStringParamName());
