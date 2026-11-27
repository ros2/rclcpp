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

/**
 * Stress tests for the TimeSource::destroy_clock_sub deadlock fix.
 *
 * Exercises the conditions from ros2/rclcpp#2962: rapid node
 * construction/destruction with use_sim_time enabled. If the
 * deadlock is present, these tests hang (caught by the test timeout).
 */

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;

class TestTimeSourceDeadlock : public ::testing::Test
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
};

/// Reproducer for ros2/rclcpp#2962: rapid construction and destruction
/// of a node with use_sim_time=true. The clock executor thread must
/// be joined without deadlock.
TEST_F(TestTimeSourceDeadlock, rapid_construct_destroy_sim_time)
{
  for (int i = 0; i < 50; ++i) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = std::make_shared<rclcpp::Node>(
      "deadlock_test_" + std::to_string(i), "/test_ns", opts);

    // Immediately destroy -- the clock executor thread must be
    // joined without deadlock.
    node.reset();
  }
}

/// Create and destroy nodes with use_sim_time from multiple threads
/// to stress the lock/join interaction.
TEST_F(TestTimeSourceDeadlock, concurrent_construct_destroy)
{
  const int num_threads = 4;
  const int iterations = 10;
  std::vector<std::future<void>> futures;

  for (int t = 0; t < num_threads; ++t) {
    futures.push_back(
      std::async(
        std::launch::async, [t, iterations]() {
          for (int i = 0; i < iterations; ++i) {
            rclcpp::NodeOptions opts;
            opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
            auto node = std::make_shared<rclcpp::Node>(
              "concurrent_test_" + std::to_string(t) + "_" + std::to_string(i),
              "/test_ns", opts);
            node.reset();
          }
        }));
  }

  for (auto & f : futures) {
    ASSERT_NO_THROW(f.get());
  }
}

/// Toggle use_sim_time parameter while spinning -- exercises the
/// create_clock_sub / destroy_clock_sub interplay.
TEST_F(TestTimeSourceDeadlock, toggle_use_sim_time_while_spinning)
{
  auto node = std::make_shared<rclcpp::Node>("toggle_test", "/test_ns");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin in background
  auto spin_future = std::async(
    std::launch::async, [&executor]() {
      executor.spin();
    });

  // Toggle use_sim_time several times
  for (int i = 0; i < 10; ++i) {
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    std::this_thread::sleep_for(10ms);
    node->set_parameter(rclcpp::Parameter("use_sim_time", false));
    std::this_thread::sleep_for(10ms);
  }

  executor.cancel();
  spin_future.get();
}

/// Verify the clock thread is properly cleaned up after destruction.
/// After destroying a sim-time node, no threads from its clock
/// executor should be lingering.
TEST_F(TestTimeSourceDeadlock, no_lingering_thread_after_destroy)
{
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = std::make_shared<rclcpp::Node>(
      "linger_test", "/test_ns", opts);
    // Node destroyed here -- clock thread must be fully joined
  }

  // If the thread were dangling, subsequent operations on the
  // same context would be unreliable. Create another node to
  // verify the context is clean.
  auto node2 = std::make_shared<rclcpp::Node>("after_linger", "/test_ns");
  ASSERT_NE(node2, nullptr);
}

/// Rapid construct/destroy while /clock traffic is flowing, so the
/// clock executor thread is actively dispatching callbacks while
/// destroy_clock_sub() runs -- the exact window from ros2/rclcpp#2962.
TEST_F(TestTimeSourceDeadlock, rapid_construct_destroy_with_clock_traffic)
{
  auto clock_pub_node = std::make_shared<rclcpp::Node>("clock_pub", "/test_ns");
  auto clock_pub = clock_pub_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  std::atomic<bool> stop{false};
  auto pub_thread = std::thread(
    [&]() {
      rclcpp::executors::SingleThreadedExecutor pub_executor;
      pub_executor.add_node(clock_pub_node);
      auto timer = clock_pub_node->create_wall_timer(
        5ms, [&]() {
          rosgraph_msgs::msg::Clock msg;
          msg.clock.sec = 1;
          clock_pub->publish(msg);
        });
      while (!stop.load()) {
        pub_executor.spin_some();
        std::this_thread::sleep_for(1ms);
      }
    });

  for (int i = 0; i < 50; ++i) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = std::make_shared<rclcpp::Node>(
      "traffic_test_" + std::to_string(i), "/test_ns", opts);
    node.reset();
  }

  stop.store(true);
  pub_thread.join();
}

/// Regression test for the detach/create race flagged in review of
/// ros2/rclcpp#3412: a post_set_parameters(true) landing between
/// destroy_clock_sub() returning and detachNode() clearing the node
/// interfaces could start a fresh clock executor thread against null
/// interfaces. Hammer detachNode() while another thread toggles
/// use_sim_time: the detach must never crash or hang, and the
/// TimeSource must still be usable (re-attachable) afterwards.
/// Without the detaching_ flag this window is open; with it,
/// create_clock_sub() no-ops for the whole teardown.
TEST_F(TestTimeSourceDeadlock, detach_races_sim_time_enable)
{
  for (int iter = 0; iter < 20; ++iter) {
    auto node = std::make_shared<rclcpp::Node>(
      "detach_race_" + std::to_string(iter), "/test_ns");
    rclcpp::TimeSource time_source(rclcpp::QoS(10), true);
    time_source.attachNode(node);

    std::atomic<bool> stop{false};
    auto toggler = std::thread(
      [&]() {
        bool enable = true;
        while (!stop.load()) {
          try {
            node->set_parameter(rclcpp::Parameter("use_sim_time", enable));
          } catch (...) {
            // detachNode() may unregister the parameter callbacks
            // mid-call; that is expected here.
          }
          enable = !enable;
        }
      });

    // Detach while the toggler hammers use_sim_time: no clock executor
    // thread may be created past this point.
    ASSERT_NO_THROW(time_source.detachNode());
    stop.store(true);
    toggler.join();

    // Re-attaching after a detach must still work (the detaching flag
    // is cleared by attachNode) and a clean detach must follow.
    ASSERT_NO_THROW(time_source.attachNode(node));
    ASSERT_NO_THROW(node->set_parameter(rclcpp::Parameter("use_sim_time", true)));
    ASSERT_NO_THROW(time_source.detachNode());
  }
}
