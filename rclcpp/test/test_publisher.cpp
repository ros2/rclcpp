// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"

#include "test_msgs/msg/empty.hpp"

class TestPublisher : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

protected:
  void initialize(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns", node_options);
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

struct TestParameters
{
  TestParameters(rclcpp::QoS qos, const std::string & description)
  : qos(qos), description(description) {}
  rclcpp::QoS qos;
  std::string description;
};

std::ostream & operator<<(std::ostream & out, const TestParameters & params)
{
  out << params.description;
  return out;
}

class TestPublisherInvalidIntraprocessQos
  : public TestPublisher,
  public ::testing::WithParamInterface<TestParameters>
{};

class TestPublisherSub : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    subnode = node->create_sub_node("sub_ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr subnode;
};

/*
   Testing publisher construction and destruction.
 */
TEST_F(TestPublisher, construction_and_destruction) {
  initialize();
  using test_msgs::msg::Empty;
  {
    auto publisher = node->create_publisher<Empty>("topic", 42);
    (void)publisher;
  }

  {
    ASSERT_THROW(
    {
      auto publisher = node->create_publisher<Empty>("invalid_topic?", 42);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}

/*
   Testing publisher creation signatures.
 */
TEST_F(TestPublisher, various_creation_signatures) {
  initialize();
  using test_msgs::msg::Empty;
  {
    auto publisher = node->create_publisher<Empty>("topic", 42);
    (void)publisher;
  }
  {
    auto publisher = node->create_publisher<Empty>("topic", rclcpp::QoS(42));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<Empty>("topic", rclcpp::QoS(rclcpp::KeepLast(42)));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<Empty>("topic", rclcpp::QoS(rclcpp::KeepAll()));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<Empty>("topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
  {
    auto publisher =
      rclcpp::create_publisher<Empty>(node, "topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
}

/*
   Testing for serialized publishers.
 */
TEST_F(TestPublisher, test_is_serialized) {
  initialize();

  using test_msgs::msg::Empty;
  {
    auto publisher = node->create_publisher<Empty>("topic", 42);
    EXPECT_FALSE(publisher->is_serialized());
  }
  {
    auto publisher = rclcpp::create_publisher<Empty>(node, "topic", 42, rclcpp::PublisherOptions());
    EXPECT_FALSE(publisher->is_serialized());
  }
  {
    auto ts = *rosidl_typesupport_cpp::get_message_type_support_handle<Empty>();
    auto publisher = rclcpp::create_publisher<rcl_serialized_message_t>(
      node, "topic", ts, 42, rclcpp::PublisherOptions());
    EXPECT_TRUE(publisher->is_serialized());
  }
  {
    auto ts = *rosidl_typesupport_cpp::get_message_type_support_handle<Empty>();
    auto publisher = rclcpp::create_publisher<rclcpp::SerializedMessage>(
      node, "topic", ts, 42, rclcpp::PublisherOptions());
    EXPECT_TRUE(publisher->is_serialized());
  }
}

/*
   Testing publisher with intraprocess enabled and invalid QoS
 */
TEST_P(TestPublisherInvalidIntraprocessQos, test_publisher_throws) {
  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));
  rclcpp::QoS qos = GetParam().qos;
  using test_msgs::msg::Empty;
  {
    ASSERT_THROW(
      {auto publisher = node->create_publisher<Empty>("topic", qos);},
      std::invalid_argument);
  }
}

static std::vector<TestParameters> invalid_qos_profiles()
{
  std::vector<TestParameters> parameters;

  parameters.reserve(2);
  parameters.push_back(
    TestParameters(
      rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(),
      "transient_local_qos"));
  parameters.push_back(
    TestParameters(
      rclcpp::QoS(rclcpp::KeepAll()),
      "keep_all_qos"));

  return parameters;
}

INSTANTIATE_TEST_CASE_P(
  TestPublisherThrows, TestPublisherInvalidIntraprocessQos,
  ::testing::ValuesIn(invalid_qos_profiles()),
  ::testing::PrintToStringParamName());

/*
   Testing publisher construction and destruction for subnodes.
 */
TEST_F(TestPublisherSub, construction_and_destruction) {
  using test_msgs::msg::Empty;
  {
    auto publisher = subnode->create_publisher<Empty>("topic", 42);

    EXPECT_STREQ(publisher->get_topic_name(), "/ns/sub_ns/topic");
  }

  {
    auto publisher = subnode->create_publisher<Empty>("/topic", 42);

    EXPECT_STREQ(publisher->get_topic_name(), "/topic");
  }

  {
    ASSERT_THROW(
    {
      auto publisher = subnode->create_publisher<Empty>("invalid_topic?", 42);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}
