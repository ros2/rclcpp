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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

class TestPublisher : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

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

static constexpr rmw_qos_profile_t invalid_qos_profile()
{
  rmw_qos_profile_t profile = rmw_qos_profile_default;
  profile.depth = 1;
  profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  return profile;
}

/*
   Testing publisher construction and destruction.
 */
TEST_F(TestPublisher, construction_and_destruction) {
  initialize();
  using rcl_interfaces::msg::IntraProcessMessage;
  {
    auto publisher = node->create_publisher<IntraProcessMessage>("topic", 42);
    (void)publisher;
  }

  {
    ASSERT_THROW({
      auto publisher = node->create_publisher<IntraProcessMessage>("invalid_topic?", 42);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}

/*
   Testing publisher creation signatures.
 */
TEST_F(TestPublisher, various_creation_signatures) {
  initialize();
  using rcl_interfaces::msg::IntraProcessMessage;
  {
    auto publisher = node->create_publisher<IntraProcessMessage>("topic", 42);
    (void)publisher;
  }
  {
    auto publisher = node->create_publisher<IntraProcessMessage>("topic", rclcpp::QoS(42));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<IntraProcessMessage>("topic", rclcpp::QoS(rclcpp::KeepLast(42)));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<IntraProcessMessage>("topic", rclcpp::QoS(rclcpp::KeepAll()));
    (void)publisher;
  }
  {
    auto publisher =
      node->create_publisher<IntraProcessMessage>("topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
  {
    auto publisher =
      rclcpp::create_publisher<IntraProcessMessage>(node, "topic", 42, rclcpp::PublisherOptions());
    (void)publisher;
  }
  // Now deprecated functions.
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  {
    auto publisher = node->create_publisher<IntraProcessMessage>("topic");
    (void)publisher;
  }
  {
    auto publisher = node->create_publisher<IntraProcessMessage>(
      "topic",
      42,
      std::make_shared<std::allocator<IntraProcessMessage>>());
    (void)publisher;
  }
  {
    auto publisher = node->create_publisher<IntraProcessMessage>("topic", rmw_qos_profile_default);
    (void)publisher;
  }
  {
    auto publisher = node->create_publisher<IntraProcessMessage>(
      "topic",
      rmw_qos_profile_default,
      std::make_shared<std::allocator<IntraProcessMessage>>());
    (void)publisher;
  }
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
}

/*
   Testing publisher with intraprocess enabled and invalid QoS
 */
TEST_F(TestPublisher, intraprocess_with_invalid_qos) {
  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));
  rmw_qos_profile_t qos = invalid_qos_profile();
  using rcl_interfaces::msg::IntraProcessMessage;
  {
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
    ASSERT_THROW(
      {auto publisher = node->create_publisher<IntraProcessMessage>("topic", qos);},
      rclcpp::exceptions::InvalidParametersException);
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
  }
}

/*
   Testing publisher construction and destruction for subnodes.
 */
TEST_F(TestPublisherSub, construction_and_destruction) {
  using rcl_interfaces::msg::IntraProcessMessage;
  {
    auto publisher = subnode->create_publisher<IntraProcessMessage>("topic", 42);

    EXPECT_STREQ(publisher->get_topic_name(), "/ns/sub_ns/topic");
  }

  {
    auto publisher = subnode->create_publisher<IntraProcessMessage>("/topic", 42);

    EXPECT_STREQ(publisher->get_topic_name(), "/topic");
  }

  {
    ASSERT_THROW({
      auto publisher = subnode->create_publisher<IntraProcessMessage>("invalid_topic?", 42);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}
