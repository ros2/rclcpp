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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"

#include "rosgraph_msgs/msg/clock.hpp"

#include "./executor_types.hpp"

using namespace std::chrono_literals;

// Timers on simulation time must keep firing when /clock jumps, backward as
// well as forward, and when simulation time is activated after the timer was
// created. The test publishes /clock itself so that the jumps are exact.
template<typename T>
class TestExecutorsTimeJump : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    clock_node = std::make_shared<rclcpp::Node>("clock_publisher");
    clock_publisher =
      clock_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  }

  void TearDown() override
  {
    executor.cancel();
    if (spinner.joinable()) {
      spinner.join();
    }
    timer.reset();
    node.reset();
    clock_publisher.reset();
    clock_node.reset();
  }

  void create_node_and_spin(bool use_sim_time)
  {
    rclcpp::NodeOptions options;
    if (use_sim_time) {
      options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    }
    node = std::make_shared<rclcpp::Node>("timer_node", options);
    timer = node->create_timer(100ms, [this]() {fires++;});
    executor.add_node(node);
    spinner = std::thread([this]() {executor.spin();});
  }

  void wait_for_clock_subscription()
  {
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (clock_publisher->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(1ms);
    }
    ASSERT_NE(0u, clock_publisher->get_subscription_count());
  }

  // Publishes sim time from start_ns on in 10ms steps paced by real time and
  // returns how often the timer fired meanwhile. Each second of sim time
  // should produce about ten timer calls.
  int play_clock(int64_t start_ns, std::chrono::nanoseconds sim_duration)
  {
    const int fires_before = fires;
    const auto step = std::chrono::nanoseconds(10ms).count();
    rosgraph_msgs::msg::Clock msg;
    for (int64_t elapsed = 0; elapsed < sim_duration.count(); elapsed += step) {
      msg.clock = rclcpp::Time(start_ns + elapsed);
      clock_publisher->publish(msg);
      std::this_thread::sleep_for(5ms);
    }
    return fires - fires_before;
  }

  std::shared_ptr<rclcpp::Node> clock_node;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher;
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::TimerBase::SharedPtr timer;
  std::atomic<int> fires = 0;
  std::thread spinner;
  T executor;
};

// The EventsExecutor does not support simulation time, see
// https://github.com/ros2/rclcpp/issues/2457.
using TimeJumpExecutorTypes =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::executors::EventsCBGExecutor>;

TYPED_TEST_SUITE(TestExecutorsTimeJump, TimeJumpExecutorTypes, ExecutorTypeNames);

TYPED_TEST(TestExecutorsTimeJump, timer_runs_through_backward_jump) {
  this->create_node_and_spin(true);
  this->wait_for_clock_subscription();

  const auto second = std::chrono::nanoseconds(1s).count();
  EXPECT_GE(this->play_clock(1000 * second, 1s), 5);
  // backward by far more than one timer period
  EXPECT_GE(this->play_clock(900 * second, 1s), 5);
  EXPECT_GE(this->play_clock(1004 * second, 1s), 5);
}

TYPED_TEST(TestExecutorsTimeJump, timer_runs_after_sim_time_activation) {
  this->create_node_and_spin(false);
  // the timer is scheduled on system time until the first /clock sample arrives
  this->node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  this->wait_for_clock_subscription();

  const auto second = std::chrono::nanoseconds(1s).count();
  EXPECT_GE(this->play_clock(1000 * second, 1s), 5);
}
