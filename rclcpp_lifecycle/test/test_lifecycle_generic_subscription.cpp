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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class TestLifecycleGenericSubscription : public ::testing::Test
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
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_lifecycle_generic_subscription");
};

TEST_F(TestLifecycleGenericSubscription, supports_serialized_message_callback_types)
{
  constexpr char topic_type[] = "test_msgs/msg/Strings";
  const auto qos = rclcpp::QoS(1);

  auto unique_ptr_subscription = node_->create_generic_subscription(
    "unique_ptr_callback",
    topic_type,
    qos,
    [](std::unique_ptr<rclcpp::SerializedMessage>) {});
  EXPECT_NE(nullptr, unique_ptr_subscription);

  auto with_info_subscription = node_->create_generic_subscription(
    "callback_with_message_info",
    topic_type,
    qos,
    [](const rclcpp::SerializedMessage &, const rclcpp::MessageInfo &) {});
  EXPECT_NE(nullptr, with_info_subscription);
}
