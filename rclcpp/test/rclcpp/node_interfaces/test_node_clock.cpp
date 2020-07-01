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

#include "rclcpp/node_interfaces/node_clock.hpp"
#include "rclcpp/node.hpp"

class TestNodeClock : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestNodeClock, construct_from_node)
{
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("node", "ns");

  // This dynamic cast is not necessary for the unittest itself, but instead is used to ensure
  // the proper type is being tested and covered.
  auto * node_clock =
    dynamic_cast<rclcpp::node_interfaces::NodeClock *>(node->get_node_clock_interface().get());
  ASSERT_NE(nullptr, node_clock);
  EXPECT_NE(nullptr, node_clock->get_clock());

  const auto * const_node_clock = node_clock;
  EXPECT_NE(nullptr, const_node_clock->get_clock());
}
