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
 * Integration tests for CallbackGroup entity cleanup.
 *
 * Verifies that expired weak_ptr entries are properly cleaned up when
 * cleanup is deferred to collect_all_ptrs (instead of running on every
 * add_* call).
 */

#include <chrono>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/empty.hpp"
#include "test_msgs/srv/empty.hpp"

using namespace std::chrono_literals;

class TestCallbackGroupEntityCleanup : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_node", "/test_ns");
  }

  void TearDown() override
  {
    node_.reset();
  }

  /// Trigger collect_all_ptrs to run cleanup on the given callback group.
  void trigger_collection(rclcpp::CallbackGroup::SharedPtr cbg)
  {
    cbg->collect_all_ptrs(
      [](const rclcpp::SubscriptionBase::SharedPtr &) {},
      [](const rclcpp::ServiceBase::SharedPtr &) {},
      [](const rclcpp::ClientBase::SharedPtr &) {},
      [](const rclcpp::TimerBase::SharedPtr &) {},
      [](const rclcpp::Waitable::SharedPtr &) {});
  }

  rclcpp::Node::SharedPtr node_;
};

/// After adding and then destroying timers, collect_all_ptrs should
/// compact the expired entries, reducing size().
TEST_F(TestCallbackGroupEntityCleanup, expired_timers_cleaned_on_collect)
{
  auto cbg = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Add 100 timers
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  for (int i = 0; i < 100; ++i) {
    timers.push_back(node_->create_wall_timer(1h, []() {}, cbg));
  }
  EXPECT_EQ(cbg->size(), 100u);

  // Destroy half the timers (the weak_ptrs in the callback group expire)
  timers.erase(timers.begin(), timers.begin() + 50);

  // Before collection, size() still reports 100 (expired entries remain)
  EXPECT_EQ(cbg->size(), 100u);

  // Trigger collect_all_ptrs — should compact expired entries
  trigger_collection(cbg);

  // After collection, only the 50 live timers should remain
  EXPECT_EQ(cbg->size(), 50u);
}

/// After adding and destroying subscriptions, collect_all_ptrs should
/// compact the expired entries.  Each subscription may register
/// additional entities (e.g. event waitables), so we use relative sizes.
TEST_F(TestCallbackGroupEntityCleanup, expired_subscriptions_cleaned_on_collect)
{
  auto cbg = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  for (int i = 0; i < 20; ++i) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cbg;
    subs.push_back(
      node_->create_subscription<test_msgs::msg::Empty>(
        "topic_" + std::to_string(i), 10,
        [](test_msgs::msg::Empty::ConstSharedPtr) {}, opts));
  }
  const size_t size_with_all = cbg->size();
  EXPECT_GT(size_with_all, 0u);

  // Destroy all subscriptions — expired entries remain until collection
  subs.clear();
  EXPECT_EQ(cbg->size(), size_with_all);

  trigger_collection(cbg);

  EXPECT_EQ(cbg->size(), 0u);  // all cleaned up
}

/// collect_all_ptrs should only yield live entities and skip expired ones.
TEST_F(TestCallbackGroupEntityCleanup, collect_only_yields_live_entities)
{
  auto cbg = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Add 10 timers
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  for (int i = 0; i < 10; ++i) {
    timers.push_back(node_->create_wall_timer(1h, []() {}, cbg));
  }

  // Destroy odd-indexed timers (0-based: 1, 3, 5, 7, 9)
  for (int i = 9; i >= 1; i -= 2) {
    timers.erase(timers.begin() + i);
  }
  ASSERT_EQ(timers.size(), 5u);

  // Collect and count live timers
  size_t collected_count = 0;
  cbg->collect_all_ptrs(
    [](const rclcpp::SubscriptionBase::SharedPtr &) {},
    [](const rclcpp::ServiceBase::SharedPtr &) {},
    [](const rclcpp::ClientBase::SharedPtr &) {},
    [&collected_count](const rclcpp::TimerBase::SharedPtr &) {
      ++collected_count;
    },
    [](const rclcpp::Waitable::SharedPtr &) {});

  EXPECT_EQ(collected_count, 5u);
  EXPECT_EQ(cbg->size(), 5u);  // compacted
}

/// Adding many entities should not exhibit quadratic slowdown.
/// This test adds N timers and asserts the time is well under the
/// quadratic threshold.
TEST_F(TestCallbackGroupEntityCleanup, add_many_timers_is_not_quadratic)
{
  auto cbg = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  const size_t N = 5000;
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  timers.reserve(N);

  auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < N; ++i) {
    timers.push_back(node_->create_wall_timer(1h, []() {}, cbg));
  }
  auto elapsed = std::chrono::steady_clock::now() - start;
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

  EXPECT_EQ(cbg->size(), N);

  // With O(N²) the old code took ~429ms for 10K timers.
  // With O(N) we expect < 100ms for 5K timers even on slow CI.
  // Use a generous threshold to avoid flakiness.
  EXPECT_LT(ms, 5000)  << "Adding " << N << " timers took " << ms
                       << "ms — possible quadratic regression";
}

/// After interleaved add/remove cycles, collect_all_ptrs should
/// report exactly the live entities.
TEST_F(TestCallbackGroupEntityCleanup, interleaved_add_remove_cycles)
{
  auto cbg = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  std::vector<rclcpp::TimerBase::SharedPtr> timers;

  // Cycle 1: add 20, remove 10
  for (int i = 0; i < 20; ++i) {
    timers.push_back(node_->create_wall_timer(1h, []() {}, cbg));
  }
  timers.erase(timers.begin(), timers.begin() + 10);

  // Cycle 2: add 30 more
  for (int i = 0; i < 30; ++i) {
    timers.push_back(node_->create_wall_timer(1h, []() {}, cbg));
  }

  // Cycle 3: remove 15
  timers.erase(timers.begin(), timers.begin() + 15);

  // Trigger cleanup
  size_t live = 0;
  cbg->collect_all_ptrs(
    [](const rclcpp::SubscriptionBase::SharedPtr &) {},
    [](const rclcpp::ServiceBase::SharedPtr &) {},
    [](const rclcpp::ClientBase::SharedPtr &) {},
    [&live](const rclcpp::TimerBase::SharedPtr &) {++live;},
    [](const rclcpp::Waitable::SharedPtr &) {});

  EXPECT_EQ(live, timers.size());
  EXPECT_EQ(cbg->size(), timers.size());
}

/// Mixed entity types: timers + subscriptions, verify independent cleanup.
/// Subscriptions may register extra entities (event waitables), so we
/// track relative sizes.
TEST_F(TestCallbackGroupEntityCleanup, mixed_entity_types_cleanup)
{
  auto cbg = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  // Add timers and subscriptions
  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  const int N = 10;
  for (int i = 0; i < N; ++i) {
    timers.push_back(node_->create_wall_timer(1h, []() {}, cbg));
  }
  const size_t size_timers_only = cbg->size();
  EXPECT_EQ(size_timers_only, static_cast<size_t>(N));

  for (int i = 0; i < N; ++i) {
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cbg;
    subs.push_back(
      node_->create_subscription<test_msgs::msg::Empty>(
        "mix_topic_" + std::to_string(i), 10,
        [](test_msgs::msg::Empty::ConstSharedPtr) {}, opts));
  }
  const size_t size_both = cbg->size();
  const size_t sub_entities = size_both - size_timers_only;
  EXPECT_GT(sub_entities, 0u);

  // Destroy all timers, keep subscriptions
  timers.clear();
  trigger_collection(cbg);
  EXPECT_EQ(cbg->size(), sub_entities);

  // Destroy all subscriptions
  subs.clear();
  trigger_collection(cbg);
  EXPECT_EQ(cbg->size(), 0u);
}
