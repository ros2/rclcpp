// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class TestLifecycleGenericPublisher : public ::testing::Test
{
public:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

protected:
  void TearDown() override
  {
    node_->shutdown();
    node_.reset();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_lifecycle_generic_publisher");
};

TEST_F(TestLifecycleGenericPublisher, creation_and_destruction)
{
  constexpr char topic_name[] = "test_lifecycle_generic_pub_topic";
  constexpr char topic_type[] = "test_msgs/msg/Strings";
  const auto qos = rclcpp::QoS(10);

  auto publisher = node_->create_generic_publisher(topic_name, topic_type, qos);
  ASSERT_NE(nullptr, publisher);
  EXPECT_STREQ("/test_lifecycle_generic_pub_topic", publisher->get_topic_name());
}
