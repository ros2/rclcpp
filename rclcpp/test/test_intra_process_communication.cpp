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

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "test_msgs/message_fixtures.hpp"

int32_t & get_test_allocation_counter()
{
  static int32_t counter = 0;
  return counter;
}

void * custom_allocate(size_t size, void * state)
{
  fprintf(stderr, "calling custom allocate\n");
  static auto m_allocator = rcutils_get_default_allocator();

  ++get_test_allocation_counter();
  auto r = m_allocator.allocate(size, state);
  return r;
}

void * custom_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state)
{
  fprintf(stderr, "calling custom zero allocate\n");
  static auto m_allocator = rcutils_get_default_allocator();

  ++get_test_allocation_counter();
  auto r = m_allocator.zero_allocate(number_of_elements, size_of_element, state);
  return r;
}

void * custom_reallocate(void * pointer, size_t size, void * state)
{
  fprintf(stderr, "calling custom reallocate\n");
  static auto m_allocator = rcutils_get_default_allocator();

  if (pointer == nullptr) {
    ++get_test_allocation_counter();
  }

  auto r = m_allocator.reallocate(pointer, size, state);
  return r;
}

void custom_deallocate(void * pointer, void * state)
{
  fprintf(stderr, "calling custom deallocate\n");
  static auto m_allocator = rcutils_get_default_allocator();

  --get_test_allocation_counter();
  m_allocator.deallocate(pointer, state);
}

rclcpp::SerializedMessage make_serialized_string_msg(
  const std::shared_ptr<test_msgs::msg::Strings> & string_msg)
{
  auto m_allocator = rcutils_get_default_allocator();

  // add custom (de)allocator to count the references to the object
  m_allocator.allocate = &custom_allocate;
  m_allocator.deallocate = &custom_deallocate;
  m_allocator.reallocate = &custom_reallocate;
  m_allocator.zero_allocate = &custom_zero_allocate;

  rclcpp::SerializedMessage msg(m_allocator);

  static auto type_support =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Strings>();

  rclcpp::Serialization serializer(*type_support);
  EXPECT_NO_THROW(serializer.serialize_message(string_msg.get(), &msg));

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

auto get_new_context()
{
  auto context = rclcpp::Context::make_shared();
  context->init(0, nullptr);
  return context;
}

class TestPublisherSubscriptionSerialized : public ::testing::Test
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

protected:
  std::chrono::milliseconds offset_ = std::chrono::milliseconds(250);

  std::vector<TestParameters> parameters_ = {
    /*
       Testing publisher subscription count api and internal process subscription count.
       Two subscriptions in the same topic, both using intraprocess comm.
     */
    {
      {
        rclcpp::NodeOptions().use_intra_process_comms(true).start_parameter_services(false).enable_rosout(false),
        rclcpp::NodeOptions().use_intra_process_comms(true).start_parameter_services(false).enable_rosout(false)
      },
      {1u, 2u},
      1,
      "two_subscriptions_intraprocess_comm"
    },
    ///*
    //   Testing publisher subscription count api and internal process subscription count.
    //   Two subscriptions, one using intra-process comm and the other not using it.
    // */
    //{
    //  {
    //    rclcpp::NodeOptions().use_intra_process_comms(true),
    //    rclcpp::NodeOptions().use_intra_process_comms(false)
    //  },
    //  {1u, 1u},
    //  1,
    //  "two_subscriptions_one_intraprocess_one_not"
    //},
    ///*
    //   Testing publisher subscription count api and internal process subscription count.
    //   Two contexts, both using intra-process.
    // */
    //{
    //  {
    //    rclcpp::NodeOptions().use_intra_process_comms(true),
    //    rclcpp::NodeOptions().context(get_new_context()).use_intra_process_comms(true)
    //  },
    //  {1u, 1u},
    //  1,
    //  "two_subscriptions_in_two_contexts_with_intraprocess_comm"
    //},
    ///*
    //   Testing publisher subscription count api and internal process subscription count.
    //   Two contexts, both of them not using intra-process comm.
    // */
    //{
    //  {
    //    rclcpp::NodeOptions().use_intra_process_comms(false),
    //    rclcpp::NodeOptions().context(get_new_context()).use_intra_process_comms(false)
    //  },
    //  {0u, 0u},
    //  1,
    //  "two_subscriptions_in_two_contexts_without_intraprocess_comm"
    //}
  };
};

std::array<uint32_t, 2> counts;

void OnMessageSerialized(const std::shared_ptr<const rcl_serialized_message_t> msg)
{
  EXPECT_NE(msg->buffer, nullptr);
  EXPECT_GT(msg->buffer_capacity, 0u);

  static auto type_support =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Strings>();

  test_msgs::msg::Strings ros_msg;
  rclcpp::Serialization serializer(*type_support);
  EXPECT_NO_THROW(
    serializer.deserialize_message(
      static_cast<const rclcpp::SerializedMessage *>(msg.get()), &ros_msg));
  EXPECT_EQ(ros_msg.string_value.front(), '0');
  EXPECT_EQ(ros_msg.string_value[6], '6');
  EXPECT_EQ(ros_msg.string_value.back(), '9');
  ++counts[0];
}

void OnMessageConst(std::shared_ptr<const test_msgs::msg::Strings> msg)
{
  EXPECT_EQ(msg->string_value[6], '6');

  ++counts[1];
}

void OnMessageUniquePtr(std::unique_ptr<test_msgs::msg::Strings> msg)
{
  EXPECT_EQ(msg->string_value.back(), '9');

  ++counts[1];
}

void OnMessage(std::shared_ptr<test_msgs::msg::Strings> msg)
{
  EXPECT_EQ(msg->string_value.front(), '0');
  EXPECT_EQ(msg->string_value[6], '6');
  EXPECT_EQ(msg->string_value.back(), '\0');

  ++counts[1];
}

TEST_F(TestPublisherSubscriptionSerialized, publish_serialized)
{
  get_test_allocation_counter() = 0;

  for (const auto & parameter : parameters_) {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
        "my_node",
        "/ns",
        parameter.node_options[0]);
    auto publisher = node->create_publisher<test_msgs::msg::Strings>("/topic", 10);

    //auto sub_shared = node->create_subscription<test_msgs::msg::Strings>(
    //    "/topic", 10,
    //    &OnMessage);
    //auto sub_unique = node->create_subscription<test_msgs::msg::Strings>(
    //    "/topic", 10,
    //    &OnMessageUniquePtr);
    //auto sub_const_shared = node->create_subscription<test_msgs::msg::Strings>(
    //    "/topic", 10,
    //    &OnMessageConst);
    auto sub_serialized = node->create_subscription<test_msgs::msg::Strings>(
        "/topic", 10,
        &OnMessageSerialized);

    rclcpp::sleep_for(offset_);

    counts.fill(0);
    std::shared_ptr<test_msgs::msg::Strings> string_msg = get_messages_strings()[3];

    for (size_t i = 0; i < parameter.runs; i++) {
      auto msg0 = make_serialized_string_msg(string_msg);

      auto unique_string_msg = std::make_unique<test_msgs::msg::Strings>(*string_msg);

      {
        auto unique_serialized_msg = std::make_unique<rclcpp::SerializedMessage>(std::move(msg0));
        //publisher->publish(std::move(unique_serialized_msg));
      }
      //publisher->publish(*string_msg);
      publisher->publish(std::move(unique_string_msg));
    }
    for (uint32_t i = 0; i < 3; ++i) {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(offset_);
    }

    rclcpp::spin_some(node);

    if (parameter.runs == 1) {
      EXPECT_EQ(counts[0], 3u);  // count serialized message callbacks
      EXPECT_EQ(counts[1], 9u);  // count unique + shared message callbacks
    }
  }
  EXPECT_EQ(get_test_allocation_counter(), 0);
}

//TEST_P(TestPublisherSubscriptionSerialized, publish_serialized_generic)
//{
//  get_test_allocation_counter() = 0;
//
//  TestParameters parameters = GetParam();
//  {
//    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
//      "my_node",
//      "/ns",
//      parameters.node_options[0]);
//    auto publisher = rclcpp::create_publisher<rcl_serialized_message_t>(
//      node,
//      "/topic",
//      *rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Strings>(),
//      rclcpp::QoS(10));
//
//    auto sub_gen_serialized = rclcpp::create_subscription<rcl_serialized_message_t>(
//      node,
//      "/topic",
//      *rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::Strings>(),
//      rclcpp::QoS(10),
//      &OnMessageSerialized);
//
//    auto sub_shared = node->create_subscription<test_msgs::msg::Strings>(
//      "/topic", 10,
//      &OnMessage);
//    auto sub_unique = node->create_subscription<test_msgs::msg::Strings>(
//      "/topic", 10,
//      &OnMessageUniquePtr);
//    auto sub_const_shared = node->create_subscription<test_msgs::msg::Strings>(
//      "/topic", 10,
//      &OnMessageConst);
//    auto sub_serialized = node->create_subscription<test_msgs::msg::Strings>(
//      "/topic", 10,
//      &OnMessageSerialized);
//
//    rclcpp::sleep_for(offset);
//
//    counts.fill(0);
//    auto stringMsg = get_messages_strings()[3];
//
//    for (size_t i = 0; i < parameters.runs; i++) {
//      auto msg0 = make_serialized_string_msg(stringMsg);
//
//      publisher->publish(std::make_unique<rcl_serialized_message_t>(msg0));
//    }
//    rclcpp::spin_some(node);
//    rclcpp::sleep_for(offset);
//
//    rclcpp::spin_some(node);
//  }
//
//  if (parameters.runs == 1) {
//    EXPECT_EQ(counts[0], 2u);
//    EXPECT_EQ(counts[1], 3u);
//  }
//
//  EXPECT_EQ(get_test_allocation_counter(), 0);
//}



std::vector<TestParameters> setRuns(const std::vector<TestParameters> & in, const size_t runs)
{
  std::vector<TestParameters> out = in;
  for (auto & p : out) {
    p.runs = runs;
  }
  return out;
}

//INSTANTIATE_TEST_CASE_P(
//  TestWithDifferentNodeOptions, TestPublisherSubscriptionSerialized,
//  ::testing::ValuesIn(parameters),
//  ::testing::PrintToStringParamName());

//INSTANTIATE_TEST_CASE_P(
//  TestWithDifferentNodeOptions1000Runs, TestPublisherSubscriptionSerialized,
//  ::testing::ValuesIn(setRuns(parameters, 1000)),
//  ::testing::PrintToStringParamName());
