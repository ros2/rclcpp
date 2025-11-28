// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"

#include "./waitable_test_helpers.hpp"

class TestIntraProcessWaitable : public ::testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}
  static void TearDownTestCase() {rclcpp::shutdown();}
};

TEST_F(TestIntraProcessWaitable, test_that_waitable_stays_ready_after_second_wait) {
  auto node = std::make_shared<rclcpp::Node>(
    "test_node",
    rclcpp::NodeOptions().use_intra_process_comms(true));

  using test_msgs::msg::Empty;
  auto sub = node->create_subscription<Empty>("test_topic", 10, [](const Empty &) {});
  auto pub = node->create_publisher<Empty>("test_topic", 10);

  auto make_sub_intra_process_waitable_ready = [pub]() {
      pub->publish(Empty());
    };

  rclcpp::test::waitables::do_test_that_waitable_stays_ready_after_second_wait(
    sub->get_intra_process_waitable(),
    make_sub_intra_process_waitable_ready,
    true /* expected_to_stay_ready */);
}
