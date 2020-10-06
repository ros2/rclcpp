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

#include <chrono>
#include <memory>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

using namespace std::chrono_literals;

class TestStaticSingleThreadedExecutor : public ::testing::Test
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

TEST_F(TestStaticSingleThreadedExecutor, check_unimplemented) {
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  executor.add_node(node);

  EXPECT_THROW(executor.spin_some(), rclcpp::exceptions::UnimplementedError);
  EXPECT_THROW(executor.spin_once(0ns), rclcpp::exceptions::UnimplementedError);
}
