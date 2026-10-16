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

#include <chrono>
#include <memory>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

using namespace std::chrono_literals;

// shutdown() is protected; the destructor is its only public caller.
class ShutdownExecutor : public rclcpp::executors::EventsCBGExecutor
{
public:
  using rclcpp::executors::EventsCBGExecutor::shutdown;
};

class TestEventsCBGExecutorShutdown : public testing::Test
{
protected:
  static void SetUpTestCase() {rclcpp::init(0, nullptr);}

  static void TearDownTestCase() {rclcpp::shutdown();}

  static bool is_associated(const rclcpp::Node::SharedPtr & node)
  {
    return node->get_node_base_interface()->get_associated_with_executor_atomic().load();
  }

  // Puts the entity in the state its removal rejects: still listed by the executor, but no
  // longer flagged as associated with one. Removing it through another executor leaves it the
  // same way, since remove_node() checks the flag and not membership.
  static void disassociate(const rclcpp::Node::SharedPtr & node)
  {
    node->get_node_base_interface()->get_associated_with_executor_atomic().store(false);
  }

  static void disassociate(const rclcpp::CallbackGroup::SharedPtr & group)
  {
    group->get_associated_with_executor_atomic().store(false);
  }
};

/*
 * shutdown() must tear the executor down even when a node and a callback group can no longer
 * be removed: it does not throw, a second call is a no-op, and the node that could be removed
 * is free for another executor.
 */
TEST_F(TestEventsCBGExecutorShutdown, shutdown_completes_when_removals_fail)
{
  auto stale_node = std::make_shared<rclcpp::Node>("stale_node");
  auto other_node = std::make_shared<rclcpp::Node>("other_node");
  // A registered timer is what the executor's callback group cache hands back to its timer
  // manager on teardown.
  auto timer = stale_node->create_wall_timer(1s, []() {});
  auto stale_group = other_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);

  ShutdownExecutor executor;
  executor.add_node(stale_node);
  executor.add_node(other_node);
  executor.add_callback_group(stale_group, other_node->get_node_base_interface());
  ASSERT_FALSE(executor.get_all_callback_groups().empty());

  disassociate(stale_node);
  disassociate(stale_group);

  EXPECT_NO_THROW(executor.shutdown());
  EXPECT_NO_THROW(executor.shutdown());
  EXPECT_FALSE(is_associated(other_node));

  rclcpp::executors::EventsCBGExecutor next_executor;
  EXPECT_NO_THROW(next_executor.add_node(other_node));
}

/*
 * Destroying the executor in the same state must neither terminate the process nor touch
 * the timer manager after it is gone. The nodes outlive the executor so that the callback
 * group cache does run its timer cleanup. Without a sanitizer only the released node is
 * observable; the use-after-free shows up under AddressSanitizer.
 */
TEST_F(TestEventsCBGExecutorShutdown, destructor_completes_when_removals_fail)
{
  auto stale_node = std::make_shared<rclcpp::Node>("stale_node");
  auto other_node = std::make_shared<rclcpp::Node>("other_node");
  auto timer = stale_node->create_wall_timer(1s, []() {});

  {
    rclcpp::executors::EventsCBGExecutor executor;
    executor.add_node(stale_node);
    executor.add_node(other_node);
    ASSERT_FALSE(executor.get_all_callback_groups().empty());

    disassociate(stale_node);
  }

  EXPECT_FALSE(is_associated(other_node));
}
