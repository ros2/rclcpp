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

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "./executor_types.hpp"

using namespace std::chrono_literals;

template<typename T>
class TestExecutorsCbgDestroyed: public ::testing::Test
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

TYPED_TEST_SUITE(TestExecutorsCbgDestroyed, ExecutorTypes, ExecutorTypeNames);

TYPED_TEST(TestExecutorsCbgDestroyed, doesnt_segfault)
{
  auto node = rclcpp::Node::make_shared("test_node");
  auto log = node->get_logger();

  using ExecutorType = TypeParam;
  ExecutorType executor;

  auto spinThread = std::thread([&] {
    RCLCPP_INFO(log, "add_node");
    executor.add_node(node);
    RCLCPP_INFO(log, "spin");
    executor.spin();
    RCLCPP_INFO(log, "remove_node");
    executor.remove_node(node);
  });

  while (!executor.is_spinning()) {
    std::this_thread::sleep_for(1ms); 
  }

  {
    RCLCPP_INFO(log, "BEGIN");
    auto cg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto timer = node->create_timer(1s, [&log] { RCLCPP_INFO(log, "timer"); }, cg);  // SIGSEGV
    // auto timer = node->create_timer(1s, [&log] { RCLCPP_INFO(log, "timer"); }, std::move(cg)); // OK
    std::this_thread::sleep_for(100ms);  // wait a bit for the executor to catch up
    RCLCPP_INFO(log, "END");             // timer and subscribers go out of scope here
  }

  executor.cancel();
  if (spinThread.joinable()) {
    spinThread.join();
  }
}
