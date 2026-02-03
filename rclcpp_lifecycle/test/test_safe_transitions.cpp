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

/**
 * @brief Test node that can be configured to fail specific transitions
 */
class TestableLifecycleNode : public LifecycleNodeEnhanced
{
public:
  explicit TestableLifecycleNode(const std::string & name)
  : LifecycleNodeEnhanced(name),
    fail_configure_(false),
    fail_activate_(false),
    fail_deactivate_(false),
    fail_cleanup_(false)
  {
  }

  void set_fail_configure(bool fail) { fail_configure_ = fail; }
  void set_fail_activate(bool fail) { fail_activate_ = fail; }
  void set_fail_deactivate(bool fail) { fail_deactivate_ = fail; }
  void set_fail_cleanup(bool fail) { fail_cleanup_ = fail; }

protected:
  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const State &) override
  {
    if (fail_configure_) {
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const State &) override
  {
    if (fail_activate_) {
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const State &) override
  {
    if (fail_deactivate_) {
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const State &) override
  {
    if (fail_cleanup_) {
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  bool fail_configure_;
  bool fail_activate_;
  bool fail_deactivate_;
  bool fail_cleanup_;
};

class SafeTransitionsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<TestableLifecycleNode>("test_safe_transitions");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<TestableLifecycleNode> node_;
};

TEST_F(SafeTransitionsTest, SafeConfigureSuccess)
{
  ASSERT_TRUE(node_->is_unconfigured());

  auto result = node_->safe_configure();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.error_message.empty());
  EXPECT_TRUE(node_->is_inactive());
}

TEST_F(SafeTransitionsTest, SafeConfigureFailure)
{
  ASSERT_TRUE(node_->is_unconfigured());

  node_->set_fail_configure(true);
  auto result = node_->safe_configure();

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
  // Node should remain in unconfigured state or error processing
}

TEST_F(SafeTransitionsTest, SafeConfigureWrongState)
{
  // Configure successfully first
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(node_->is_inactive());

  // Try to configure again - should fail with error message
  auto result = node_->safe_configure();

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
  EXPECT_NE(result.error_message.find("not in unconfigured state"), std::string::npos);
}

TEST_F(SafeTransitionsTest, SafeActivateSuccess)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(node_->is_inactive());

  auto result = node_->safe_activate();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(node_->is_active());
}

TEST_F(SafeTransitionsTest, SafeActivateFailure)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(node_->is_inactive());

  node_->set_fail_activate(true);
  auto result = node_->safe_activate();

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(SafeTransitionsTest, SafeActivateWrongState)
{
  // Still unconfigured
  ASSERT_TRUE(node_->is_unconfigured());

  auto result = node_->safe_activate();

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
  EXPECT_NE(result.error_message.find("not in inactive state"), std::string::npos);
}

TEST_F(SafeTransitionsTest, SafeDeactivateSuccess)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(node_->is_active());

  auto result = node_->safe_deactivate();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(node_->is_inactive());
}

TEST_F(SafeTransitionsTest, SafeDeactivateFailure)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(node_->is_active());

  node_->set_fail_deactivate(true);
  auto result = node_->safe_deactivate();

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(SafeTransitionsTest, SafeDeactivateWrongState)
{
  // Only configured, not active
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(node_->is_inactive());

  auto result = node_->safe_deactivate();

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("not in active state"), std::string::npos);
}

TEST_F(SafeTransitionsTest, SafeCleanupSuccess)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(node_->is_inactive());

  auto result = node_->safe_cleanup();

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(node_->is_unconfigured());
}

TEST_F(SafeTransitionsTest, SafeCleanupFailure)
{
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(node_->is_inactive());

  node_->set_fail_cleanup(true);
  auto result = node_->safe_cleanup();

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(SafeTransitionsTest, SafeCleanupWrongState)
{
  // Active, not inactive
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(node_->is_active());

  auto result = node_->safe_cleanup();

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.error_message.find("not in inactive state"), std::string::npos);
}

TEST_F(SafeTransitionsTest, FullLifecycleWithSafeMethods)
{
  // Full lifecycle journey using safe methods
  ASSERT_TRUE(node_->is_unconfigured());

  auto result = node_->safe_configure();
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(node_->is_inactive());

  result = node_->safe_activate();
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(node_->is_active());

  result = node_->safe_deactivate();
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(node_->is_inactive());

  result = node_->safe_cleanup();
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(node_->is_unconfigured());
}

TEST_F(SafeTransitionsTest, TransitionResultHasDuration)
{
  auto result = node_->safe_configure();

  EXPECT_TRUE(result.success);
  EXPECT_GT(result.duration.count(), 0);
}

TEST_F(SafeTransitionsTest, TransitionResultHasResultingState)
{
  auto result = node_->safe_configure();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(
    result.resulting_state.id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SafeTransitionsTest, CanTransitionToValidation)
{
  // From unconfigured, can transition to inactive and finalized
  ASSERT_TRUE(node_->is_unconfigured());
  EXPECT_TRUE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE));
  EXPECT_TRUE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED));
  EXPECT_FALSE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));

  // From inactive, can transition to active, unconfigured, and finalized
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_TRUE(node_->is_inactive());
  EXPECT_TRUE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  EXPECT_TRUE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
  EXPECT_TRUE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED));

  // From active, can transition to inactive and finalized
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_TRUE(node_->is_active());
  EXPECT_TRUE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE));
  EXPECT_TRUE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED));
  EXPECT_FALSE(node_->can_transition_to(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
