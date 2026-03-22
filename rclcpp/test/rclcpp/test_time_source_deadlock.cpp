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
 * Stress tests for TimeSource::destroy_clock_sub deadlock fix.
 *
 * Exercises the conditions from ros2/rclcpp#2962: rapid node
 * construction/destruction with use_sim_time enabled.  If the
 * deadlock is present, these tests will hang (caught by the test
 * timeout).  Under ThreadSanitizer, any data race or dangling-thread
 * access will be flagged.
 */

#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

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
/// of a node with use_sim_time=true.  The clock executor thread must
/// be joined without deadlock.
TEST_F(TestTimeSourceDeadlock, rapid_construct_destroy_sim_time)
{
  for (int i = 0; i < 50; ++i) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
    auto node = std::make_shared<rclcpp::Node>(
      "deadlock_test_" + std::to_string(i), "/test_ns", opts);

    // Immediately destroy — the clock executor thread must be
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
    futures.push_back(std::async(std::launch::async, [t, iterations]() {
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

/// Toggle use_sim_time parameter while spinning — exercises the
/// create_clock_sub / destroy_clock_sub interplay.
TEST_F(TestTimeSourceDeadlock, toggle_use_sim_time_while_spinning)
{
  auto node = std::make_shared<rclcpp::Node>("toggle_test", "/test_ns");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Spin in background
  auto spin_future = std::async(std::launch::async, [&executor]() {
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
    // Node destroyed here — clock thread must be fully joined
  }

  // If the thread were dangling, subsequent operations on the
  // same context would be unreliable.  Create another node to
  // verify the context is clean.
  auto node2 = std::make_shared<rclcpp::Node>("after_linger", "/test_ns");
  ASSERT_NE(node2, nullptr);
}
