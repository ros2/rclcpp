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

/*
   Testing publisher construction and destruction.
 */
TEST_F(TestPublisher, construction_and_destruction) {
  initialize();
  using rcl_interfaces::msg::IntraProcessMessage;
  {
    auto publisher = node->create_publisher<IntraProcessMessage>("topic");
  }

  {
    ASSERT_THROW({
      auto publisher = node->create_publisher<IntraProcessMessage>("invalid_topic?");
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}

/*
   Testing publisher with intraprocess enabled and invalid QoS
 */
TEST_F(TestPublisher, intraprocess_with_invalid_qos) {
  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));
  rmw_qos_profile_t qos = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    false
  };
  using rcl_interfaces::msg::IntraProcessMessage;
  {
    ASSERT_THROW(
      {auto publisher = node->create_publisher<IntraProcessMessage>("topic", qos);},
      rclcpp::exceptions::InvalidParametersException);
  }
}

/*
   Testing publisher construction and destruction for subnodes.
 */
TEST_F(TestPublisherSub, construction_and_destruction) {
  using rcl_interfaces::msg::IntraProcessMessage;
  {
    auto publisher = subnode->create_publisher<IntraProcessMessage>("topic");

    EXPECT_STREQ(publisher->get_topic_name(), "/ns/sub_ns/topic");
  }

  {
    auto publisher = subnode->create_publisher<IntraProcessMessage>("/topic");

    EXPECT_STREQ(publisher->get_topic_name(), "/topic");
  }

  {
    ASSERT_THROW({
      auto publisher = subnode->create_publisher<IntraProcessMessage>("invalid_topic?");
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}
