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
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp_lifecycle/state_history.hpp"

using namespace std::chrono_literals;  // NOLINT(build/namespaces)
using rclcpp_lifecycle::StateHistory;
using rclcpp_lifecycle::StateHistoryEntry;
using rclcpp_lifecycle::State;

class StateHistoryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    history_ = std::make_unique<StateHistory>(100);
  }

  State make_state(uint8_t id, const std::string & label)
  {
    // Create a State object - this may need adjustment based on actual State API
    return State(id, label);
  }

  std::unique_ptr<StateHistory> history_;
};

TEST_F(StateHistoryTest, InitiallyEmpty)
{
  EXPECT_TRUE(history_->empty());
  EXPECT_EQ(history_->size(), 0u);
  EXPECT_FALSE(history_->get_current_state().has_value());
  EXPECT_FALSE(history_->get_previous_state().has_value());
}

TEST_F(StateHistoryTest, RecordSingleState)
{
  State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  history_->record_state(unconfigured, "initial");

  EXPECT_FALSE(history_->empty());
  EXPECT_EQ(history_->size(), 1u);

  auto current = history_->get_current_state();
  ASSERT_TRUE(current.has_value());
  EXPECT_EQ(current->id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  EXPECT_FALSE(history_->get_previous_state().has_value());
}

TEST_F(StateHistoryTest, RecordMultipleStates)
{
  State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  history_->record_state(unconfigured, "initial");
  history_->record_state(inactive, "configure");
  history_->record_state(active, "activate");

  EXPECT_EQ(history_->size(), 3u);

  auto current = history_->get_current_state();
  ASSERT_TRUE(current.has_value());
  EXPECT_EQ(current->id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto previous = history_->get_previous_state();
  ASSERT_TRUE(previous.has_value());
  EXPECT_EQ(previous->id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(StateHistoryTest, GetEntriesMostRecentFirst)
{
  State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  history_->record_state(unconfigured, "initial");
  history_->record_state(inactive, "configure");
  history_->record_state(active, "activate");

  auto entries = history_->get_entries();
  ASSERT_EQ(entries.size(), 3u);

  // Most recent first
  EXPECT_EQ(entries[0].state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(entries[0].trigger, "activate");

  EXPECT_EQ(entries[1].state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(entries[1].trigger, "configure");

  EXPECT_EQ(entries[2].state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(entries[2].trigger, "initial");
}

TEST_F(StateHistoryTest, GetEntriesWithLimit)
{
  State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

  history_->record_state(unconfigured, "initial");
  history_->record_state(inactive, "configure");
  history_->record_state(active, "activate");

  auto entries = history_->get_entries(2);
  ASSERT_EQ(entries.size(), 2u);

  // Most recent first, limited to 2
  EXPECT_EQ(entries[0].state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(entries[1].state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(StateHistoryTest, MaxSizeEnforced)
{
  auto small_history = std::make_unique<StateHistory>(3);

  State s1(1, "state1");
  State s2(2, "state2");
  State s3(3, "state3");
  State s4(4, "state4");
  State s5(5, "state5");

  small_history->record_state(s1, "t1");
  small_history->record_state(s2, "t2");
  small_history->record_state(s3, "t3");

  EXPECT_EQ(small_history->size(), 3u);

  small_history->record_state(s4, "t4");
  EXPECT_EQ(small_history->size(), 3u);

  auto entries = small_history->get_entries();
  EXPECT_EQ(entries[0].state.id(), 4);  // Most recent
  EXPECT_EQ(entries[2].state.id(), 2);  // Oldest still kept (s1 was removed)
}

TEST_F(StateHistoryTest, ClearHistory)
{
  State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");

  history_->record_state(unconfigured, "initial");
  history_->record_state(inactive, "configure");

  EXPECT_EQ(history_->size(), 2u);

  history_->clear();

  EXPECT_TRUE(history_->empty());
  EXPECT_EQ(history_->size(), 0u);
  EXPECT_FALSE(history_->get_current_state().has_value());
}

TEST_F(StateHistoryTest, TimeInCurrentState)
{
  State unconfigured(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");

  history_->record_state(unconfigured, "initial");

  // Wait a bit
  std::this_thread::sleep_for(50ms);

  auto duration = history_->get_time_in_current_state();
  EXPECT_GE(duration, 50ms);
  EXPECT_LT(duration, 200ms);  // Should be less than 200ms
}

TEST_F(StateHistoryTest, SetMaxSize)
{
  // Start with 5 entries
  for (int i = 1; i <= 5; ++i) {
    State s(i, "state" + std::to_string(i));
    history_->record_state(s, "t" + std::to_string(i));
  }

  EXPECT_EQ(history_->size(), 5u);

  // Reduce max size to 3
  history_->set_max_size(3);

  EXPECT_EQ(history_->size(), 3u);

  // Check that oldest entries were removed
  auto entries = history_->get_entries();
  EXPECT_EQ(entries[0].state.id(), 5);  // Most recent
  EXPECT_EQ(entries[2].state.id(), 3);  // Oldest still kept
}

TEST_F(StateHistoryTest, TimestampsAreMonotonic)
{
  State s1(1, "state1");
  State s2(2, "state2");
  State s3(3, "state3");

  history_->record_state(s1, "t1");
  std::this_thread::sleep_for(10ms);
  history_->record_state(s2, "t2");
  std::this_thread::sleep_for(10ms);
  history_->record_state(s3, "t3");

  auto entries = history_->get_entries();

  // Timestamps should be in descending order (most recent first)
  EXPECT_GT(entries[0].timestamp, entries[1].timestamp);
  EXPECT_GT(entries[1].timestamp, entries[2].timestamp);
}

// Thread safety test
TEST_F(StateHistoryTest, ThreadSafety)
{
  const int num_threads = 4;
  const int iterations = 100;

  std::vector<std::thread> threads;

  for (int t = 0; t < num_threads; ++t) {
    threads.emplace_back([this, t, iterations]() {
      for (int i = 0; i < iterations; ++i) {
        State s(t * 1000 + i, "state" + std::to_string(t * 1000 + i));
        history_->record_state(s, "trigger");

        // Also do some reads
        history_->get_current_state();
        history_->get_previous_state();
        history_->get_entries(5);
        history_->size();
      }
    });
  }

  for (auto & t : threads) {
    t.join();
  }

  // Max size is 100, so we shouldn't exceed that
  EXPECT_LE(history_->size(), 100u);
  EXPECT_GT(history_->size(), 0u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
