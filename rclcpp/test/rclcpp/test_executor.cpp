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
#include <stdexcept>
#include <string>

#include "rclcpp/executor.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

// This file tests the abstract rclcpp::Executor class.  For tests of the concrete classes
// that implement this class, please see the test/rclcpp/executors subdirectory.

class DummyExecutor : public rclcpp::Executor
{
public:
  DummyExecutor()
  : rclcpp::Executor()
  {
  }

  void spin() override
  {
  }

  void spin_nanoseconds(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
  {
    spin_node_once_nanoseconds(node, std::chrono::milliseconds(100));
  }

  rclcpp::memory_strategy::MemoryStrategy * memory_strategy_ptr()
  {
    return memory_strategy_.get();
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr local_get_node_by_group(
    rclcpp::CallbackGroup::SharedPtr group)
  {
    std::lock_guard<std::mutex> guard_{mutex_};  // only to make the TSA happy
    return get_node_by_group(weak_groups_to_nodes_, group);
  }

  rclcpp::CallbackGroup::SharedPtr local_get_group_by_timer(rclcpp::TimerBase::SharedPtr timer)
  {
    return get_group_by_timer(timer);
  }
};

class TestExecutor : public ::testing::Test
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

// Required for mocking_utils below
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, >)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, >)

TEST_F(TestExecutor, add_remove_node_thread_safe) {
  using namespace std::chrono_literals;

  // Create an Executor
  rclcpp::executors::SingleThreadedExecutor executor;

  auto future = std::async(std::launch::async, [&executor] {executor.spin();});

  // Add and remove nodes repeatedly
  // Test that this does not cause a segfault
  size_t num_nodes = 100;
  for (size_t i = 0; i < num_nodes; ++i) {
    std::ostringstream name;
    name << "node_" << i;
    auto node = std::make_shared<rclcpp::Node>(name.str());
    executor.add_node(node);
    // Sleeping here helps exaggerate the issue
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    executor.remove_node(node);
  }
  std::future_status future_status = std::future_status::timeout;
  do {
    executor.cancel();
    future_status = future.wait_for(1s);
  } while (future_status == std::future_status::timeout);
  EXPECT_EQ(future_status, std::future_status::ready);
  future.get();
}

TEST_F(TestExecutor, constructor_bad_guard_condition_init) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_guard_condition_init, RCL_RET_ERROR);
  EXPECT_THROW(
    static_cast<void>(std::make_unique<DummyExecutor>()),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestExecutor, constructor_bad_wait_set_init) {
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_wait_set_init, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    static_cast<void>(std::make_unique<DummyExecutor>()),
    std::runtime_error("Failed to create wait set in Executor constructor: error not set"));
}

TEST_F(TestExecutor, add_callback_group_twice) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  dummy.add_callback_group(cb_group, node->get_node_base_interface(), false);
  cb_group->get_associated_with_executor_atomic().exchange(false);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.add_callback_group(cb_group, node->get_node_base_interface(), false),
    std::runtime_error("Callback group was already added to executor."));
}

TEST_F(TestExecutor, add_callback_group_failed_trigger_guard_condition) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.add_callback_group(cb_group, node->get_node_base_interface(), true),
    std::runtime_error("Failed to trigger guard condition on callback group add: error not set"));
}

TEST_F(TestExecutor, remove_callback_group_null_node) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  dummy.add_callback_group(cb_group, node->get_node_base_interface(), true);

  node.reset();

  RCLCPP_EXPECT_THROW_EQ(
    dummy.remove_callback_group(cb_group, false),
    std::runtime_error("Node must not be deleted before its callback group(s)."));
}

TEST_F(TestExecutor, remove_callback_group_failed_trigger_guard_condition) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  dummy.add_callback_group(cb_group, node->get_node_base_interface(), true);

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.remove_callback_group(cb_group, true),
    std::runtime_error(
      "Failed to trigger guard condition on callback group remove: error not set"));
}

TEST_F(TestExecutor, remove_node_not_associated) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_EXPECT_THROW_EQ(
    dummy.remove_node(node->get_node_base_interface(), false),
    std::runtime_error("Node needs to be associated with an executor."));
}

TEST_F(TestExecutor, remove_node_associated_with_different_executor) {
  DummyExecutor dummy1;
  auto node1 = std::make_shared<rclcpp::Node>("node1", "ns");
  dummy1.add_node(node1->get_node_base_interface(), false);

  DummyExecutor dummy2;
  auto node2 = std::make_shared<rclcpp::Node>("node2", "ns");
  dummy2.add_node(node2->get_node_base_interface(), false);

  RCLCPP_EXPECT_THROW_EQ(
    dummy2.remove_node(node1->get_node_base_interface(), false),
    std::runtime_error("Node needs to be associated with this executor."));
}

TEST_F(TestExecutor, spin_node_once_nanoseconds) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool timer_fired = false;
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&timer_fired]() {timer_fired = true;});

  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(timer_fired);
  dummy.spin_nanoseconds(node->get_node_base_interface());
  EXPECT_TRUE(timer_fired);
}

TEST_F(TestExecutor, spin_node_some) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool timer_fired = false;
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&timer_fired]() {timer_fired = true;});

  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(timer_fired);
  dummy.spin_node_some(node);
  EXPECT_TRUE(timer_fired);
}

TEST_F(TestExecutor, spin_all_invalid_duration) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");

  RCLCPP_EXPECT_THROW_EQ(
    dummy.spin_all(std::chrono::nanoseconds(-1)),
    std::invalid_argument("max_duration must be greater than or equal to 0"));
}

TEST_F(TestExecutor, spin_some_in_spin_some) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool spin_some_in_spin_some = false;
  auto timer =
    node->create_wall_timer(
    std::chrono::milliseconds(1), [&]() {
      try {
        dummy.spin_some(std::chrono::milliseconds(1));
      } catch (const std::runtime_error & err) {
        if (err.what() == std::string("spin_some() called while already spinning")) {
          spin_some_in_spin_some = true;
        }
      }
    });

  dummy.add_node(node);
  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(spin_some_in_spin_some);
  dummy.spin_some(std::chrono::milliseconds(1));
  EXPECT_TRUE(spin_some_in_spin_some);
}

TEST_F(TestExecutor, spin_some_elapsed) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool timer_called = false;
  auto timer =
    node->create_wall_timer(
    std::chrono::milliseconds(1), [&]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      timer_called = true;
    });

  dummy.add_node(node);
  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  dummy.spin_some(std::chrono::milliseconds(1));

  ASSERT_TRUE(timer_called);
}

TEST_F(TestExecutor, spin_once_in_spin_once) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool spin_once_in_spin_once = false;
  auto timer =
    node->create_wall_timer(
    std::chrono::milliseconds(1), [&]() {
      try {
        dummy.spin_once(std::chrono::milliseconds(1));
      } catch (const std::runtime_error & err) {
        if (err.what() == std::string("spin_once() called while already spinning")) {
          spin_once_in_spin_once = true;
        }
      }
    });

  dummy.add_node(node);
  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(spin_once_in_spin_once);
  dummy.spin_once(std::chrono::milliseconds(1));
  EXPECT_TRUE(spin_once_in_spin_once);
}

TEST_F(TestExecutor, cancel_failed_trigger_guard_condition) {
  DummyExecutor dummy;

  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.cancel(),
    std::runtime_error("Failed to trigger guard condition in cancel: error not set"));
}

TEST_F(TestExecutor, set_memory_strategy_nullptr) {
  DummyExecutor dummy;

  RCLCPP_EXPECT_THROW_EQ(
    dummy.set_memory_strategy(nullptr),
    std::runtime_error("Received NULL memory strategy in executor."));
}

TEST_F(TestExecutor, set_memory_strategy) {
  DummyExecutor dummy;
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr strategy =
    std::make_shared<
    rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy<>>();

  dummy.set_memory_strategy(strategy);
  EXPECT_EQ(dummy.memory_strategy_ptr(), strategy.get());
}

TEST_F(TestExecutor, spin_once_failed_trigger_guard_condition) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&]() {});

  dummy.add_node(node);
  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.spin_once(std::chrono::milliseconds(1)),
    std::runtime_error(
      "Failed to trigger guard condition from execute_any_executable: error not set"));
}

TEST_F(TestExecutor, spin_some_fail_wait_set_clear) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&]() {});

  dummy.add_node(node);
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_wait_set_clear, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.spin_some(std::chrono::milliseconds(1)),
    std::runtime_error("Couldn't clear wait set: error not set"));
}

TEST_F(TestExecutor, spin_some_fail_wait_set_resize) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&]() {});

  dummy.add_node(node);
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_wait_set_resize, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.spin_some(std::chrono::milliseconds(1)),
    std::runtime_error("Couldn't resize the wait set: error not set"));
}

TEST_F(TestExecutor, spin_some_fail_add_handles_to_wait_set) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&]() {});

  dummy.add_node(node);
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_wait_set_add_subscription,
    RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.spin_some(std::chrono::milliseconds(1)),
    std::runtime_error("Couldn't fill wait set"));
}

TEST_F(TestExecutor, spin_some_fail_wait) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&]() {});

  dummy.add_node(node);
  auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_wait, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    dummy.spin_some(std::chrono::milliseconds(1)),
    std::runtime_error("rcl_wait() failed: error not set"));
}

TEST_F(TestExecutor, get_node_by_group_null_group) {
  DummyExecutor dummy;
  ASSERT_EQ(nullptr, dummy.local_get_node_by_group(nullptr));
}

TEST_F(TestExecutor, get_node_by_group) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  dummy.add_callback_group(cb_group, node->get_node_base_interface(), false);
  ASSERT_EQ(node->get_node_base_interface().get(), dummy.local_get_node_by_group(cb_group).get());
}

TEST_F(TestExecutor, get_node_by_group_not_found) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  ASSERT_EQ(nullptr, dummy.local_get_node_by_group(cb_group).get());
}

TEST_F(TestExecutor, get_group_by_timer_nullptr) {
  DummyExecutor dummy;
  ASSERT_EQ(nullptr, dummy.local_get_group_by_timer(nullptr));
}

TEST_F(TestExecutor, get_group_by_timer) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&]() {}, cb_group);
  dummy.add_node(node);

  ASSERT_EQ(cb_group.get(), dummy.local_get_group_by_timer(timer).get());
}

TEST_F(TestExecutor, get_group_by_timer_with_deleted_group) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&]() {}, cb_group);
  dummy.add_node(node);

  cb_group.reset();

  ASSERT_EQ(nullptr, dummy.local_get_group_by_timer(timer).get());
}

TEST_F(TestExecutor, get_group_by_timer_add_callback_group) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  rclcpp::CallbackGroup::SharedPtr cb_group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto timer =
    node->create_wall_timer(std::chrono::milliseconds(1), [&]() {}, cb_group);
  dummy.add_callback_group(cb_group, node->get_node_base_interface(), false);

  ASSERT_EQ(cb_group.get(), dummy.local_get_group_by_timer(timer).get());
}

TEST_F(TestExecutor, spin_until_future_complete_in_spin_until_future_complete) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool spin_until_future_complete_in_spin_until_future_complete = false;
  auto timer =
    node->create_wall_timer(
    std::chrono::milliseconds(1), [&]() {
      try {
        std::promise<void> promise;
        std::future<void> future = promise.get_future();
        dummy.spin_until_future_complete(future, std::chrono::milliseconds(1));
      } catch (const std::runtime_error & err) {
        if (err.what() == std::string(
          "spin_until_future_complete() called while already spinning"))
        {
          spin_until_future_complete_in_spin_until_future_complete = true;
        }
      }
    });

  dummy.add_node(node);
  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(spin_until_future_complete_in_spin_until_future_complete);
  std::promise<void> promise;
  std::future<void> future = promise.get_future();
  dummy.spin_until_future_complete(future, std::chrono::milliseconds(1));
  EXPECT_TRUE(spin_until_future_complete_in_spin_until_future_complete);
}

TEST_F(TestExecutor, spin_node_once_base_interface) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool spin_called = false;
  auto timer =
    node->create_wall_timer(
    std::chrono::milliseconds(1), [&]() {
      spin_called = true;
    });

  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(spin_called);
  dummy.spin_node_once(node->get_node_base_interface());
  EXPECT_TRUE(spin_called);
}

TEST_F(TestExecutor, spin_node_once_node) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool spin_called = false;
  auto timer =
    node->create_wall_timer(
    std::chrono::milliseconds(1), [&]() {
      spin_called = true;
    });

  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_FALSE(spin_called);
  dummy.spin_node_once(node);
  EXPECT_TRUE(spin_called);
}

TEST_F(TestExecutor, spin_until_future_complete_future_already_complete) {
  DummyExecutor dummy;
  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  std::promise<void> promise;
  std::future<void> future = promise.get_future();
  promise.set_value();
  EXPECT_EQ(
    rclcpp::FutureReturnCode::SUCCESS,
    dummy.spin_until_future_complete(future, std::chrono::milliseconds(1)));
}

TEST_F(TestExecutor, is_spinning) {
  DummyExecutor dummy;
  ASSERT_FALSE(dummy.is_spinning());

  auto node = std::make_shared<rclcpp::Node>("node", "ns");
  bool timer_called = false;
  auto timer =
    node->create_wall_timer(
    std::chrono::milliseconds(1), [&]() {
      timer_called = true;
      EXPECT_TRUE(dummy.is_spinning());
    });

  dummy.add_node(node);
  // Wait for the wall timer to have expired.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  dummy.spin_some(std::chrono::milliseconds(1));

  ASSERT_TRUE(timer_called);
}
