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

#include <memory>
#include <string>

#include "rcl/node_options.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_timers.hpp"
#include "rclcpp/rclcpp.hpp"

class TestTimer : public rclcpp::TimerBase
{
public:
  explicit TestTimer(rclcpp::Node * node)
  : TimerBase(node->get_clock(), std::chrono::nanoseconds(1),
      node->get_node_base_interface()->get_context()) {}

  void execute_callback() override {}
  bool is_steady() override {return false;}
};

TEST(TestNodeService, add_timer)
{
  rclcpp::init(0, nullptr);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but the coverage utility lcov
  // reports these functions uncovered otherwise.
  auto node_timers =
    dynamic_cast<rclcpp::node_interfaces::NodeTimers *>(node->get_node_timers_interface().get());
  ASSERT_NE(nullptr, node_timers);

  std::shared_ptr<rclcpp::Node> node2 = std::make_shared<rclcpp::Node>("node2", "ns");

  auto callback_group = node2->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto service = std::make_shared<TestTimer>(node.get());
  EXPECT_THROW(
    node_timers->add_timer(service, callback_group),
    std::runtime_error);

  rclcpp::shutdown();
}
