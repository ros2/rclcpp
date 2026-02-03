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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node_enhanced.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <memory>

using namespace rclcpp_lifecycle;

class ConvenienceMethodsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<LifecycleNodeEnhanced>("test_convenience_node");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<LifecycleNodeEnhanced> node_;
};

TEST_F(ConvenienceMethodsTest, IsUnconfiguredInitially)
{
  EXPECT_TRUE(node_->is_unconfigured());
  EXPECT_FALSE(node_->is_inactive());
  EXPECT_FALSE(node_->is_active());
  EXPECT_FALSE(node_->is_configured());
  EXPECT_FALSE(node_->is_finalized());
}

TEST_F(ConvenienceMethodsTest, IsInactiveAfterConfigure)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  EXPECT_FALSE(node_->is_unconfigured());
  EXPECT_TRUE(node_->is_inactive());
  EXPECT_FALSE(node_->is_active());
  EXPECT_TRUE(node_->is_configured());
  EXPECT_FALSE(node_->is_finalized());
}

TEST_F(ConvenienceMethodsTest, IsActiveAfterActivate)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  EXPECT_FALSE(node_->is_unconfigured());
  EXPECT_FALSE(node_->is_inactive());
  EXPECT_TRUE(node_->is_active());
  EXPECT_TRUE(node_->is_configured());
  EXPECT_FALSE(node_->is_finalized());
}

TEST_F(ConvenienceMethodsTest, IsInactiveAfterDeactivate)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

  EXPECT_FALSE(node_->is_unconfigured());
  EXPECT_TRUE(node_->is_inactive());
  EXPECT_FALSE(node_->is_active());
  EXPECT_TRUE(node_->is_configured());
  EXPECT_FALSE(node_->is_finalized());
}

TEST_F(ConvenienceMethodsTest, IsUnconfiguredAfterCleanup)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);

  EXPECT_TRUE(node_->is_unconfigured());
  EXPECT_FALSE(node_->is_inactive());
  EXPECT_FALSE(node_->is_active());
  EXPECT_FALSE(node_->is_configured());
  EXPECT_FALSE(node_->is_finalized());
}

TEST_F(ConvenienceMethodsTest, IsFinalizedAfterShutdown)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);

  EXPECT_FALSE(node_->is_unconfigured());
  EXPECT_FALSE(node_->is_inactive());
  EXPECT_FALSE(node_->is_active());
  EXPECT_FALSE(node_->is_configured());
  EXPECT_TRUE(node_->is_finalized());
}

TEST_F(ConvenienceMethodsTest, ConvenienceMethodsAreNoexcept)
{
  // These should all be noexcept
  EXPECT_TRUE(noexcept(node_->is_active()));
  EXPECT_TRUE(noexcept(node_->is_inactive()));
  EXPECT_TRUE(noexcept(node_->is_unconfigured()));
  EXPECT_TRUE(noexcept(node_->is_configured()));
  EXPECT_TRUE(noexcept(node_->is_finalized()));
}

TEST_F(ConvenienceMethodsTest, IsConfiguredTrueForBothInactiveAndActive)
{
  // Configure -> inactive state
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_TRUE(node_->is_configured());
  EXPECT_TRUE(node_->is_inactive());

  // Activate -> active state
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  EXPECT_TRUE(node_->is_configured());
  EXPECT_TRUE(node_->is_active());

  // Deactivate -> back to inactive
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  EXPECT_TRUE(node_->is_configured());
  EXPECT_TRUE(node_->is_inactive());

  // Cleanup -> unconfigured, no longer configured
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  EXPECT_FALSE(node_->is_configured());
  EXPECT_TRUE(node_->is_unconfigured());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
