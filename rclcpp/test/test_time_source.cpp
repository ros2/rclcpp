// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp"

using namespace std::chrono_literals;

class TestTimeSource : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

void spin_until_time(
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Node::SharedPtr node,
  std::chrono::nanoseconds end_time,
  bool expect_time_update)
{
  // Call spin_once on the node until either:
  // 1) We see the ros_clock's simulated time change to the expected end_time
  // -or-
  // 2) 1 second has elapsed in the real world
  // If 'expect_time_update' is True, and we timed out waiting for simulated time to
  // update, we'll have the test fail

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() < (start + 1s)) {
    if (!rclcpp::ok()) {
      break;  // Break for ctrl-c
    }

    executor.spin_once(10ms);

    if (clock->now().nanoseconds() == end_time.count()) {
      return;
    }
  }

  if (expect_time_update) {
    // If we were expecting ROS clock->now to be updated and we didn't take the early return from
    // the loop up above, that's a failure
    ASSERT_TRUE(false) << "Timed out waiting for ROS time to update";
  }
}

void spin_until_ros_time_updated(
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Node::SharedPtr node,
  rclcpp::ParameterValue value)
{
  // Similar to above:  Call spin_once until we see the clock's ros_time_is_active method
  // match the ParameterValue
  // Unlike spin_until_time, there aren't any test cases where we don't expect the value to
  // update.  In the event that the ParameterValue is not set, we'll pump messages for a full second
  // but we don't cause the test to fail

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() < (start + 1s)) {
    if (!rclcpp::ok()) {
      break;  // Break for ctrl-c
    }

    executor.spin_once(10ms);

    // In the case where we didn't intend to change the parameter, we'll still pump
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      continue;
    }

    if (clock->ros_time_is_active() == value.get<bool>()) {
      return;
    }
  }
}

void trigger_clock_changes(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<rclcpp::Clock> clock,
  bool expect_time_update = true)
{
  auto clock_pub = node->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);

  for (int i = 0; i < 5; ++i) {
    if (!rclcpp::ok()) {
      break;  // Break for ctrl-c
    }
    rosgraph_msgs::msg::Clock msg;
    msg.clock.sec = i;
    msg.clock.nanosec = 1000;
    clock_pub->publish(msg);

    // workaround.  Long-term, there can be a more elegant fix where we hook a future up
    // to a clock change callback and spin until future complete, but that's an upstream
    // change
    spin_until_time(
      clock,
      node,
      std::chrono::seconds(i) + std::chrono::nanoseconds(1000),
      expect_time_update
    );
  }
}

void set_use_sim_time_parameter(
  rclcpp::Node::SharedPtr node,
  rclcpp::ParameterValue value,
  rclcpp::Clock::SharedPtr clock)
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);

  using namespace std::chrono_literals;
  EXPECT_TRUE(parameters_client->wait_for_service(2s));
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::Parameter("use_sim_time", value)
  });
  for (auto & result : set_parameters_results) {
    EXPECT_TRUE(result.successful);
  }

  // Same as above - workaround for a little bit of asynchronus behavior.  The sim_time paramater
  // is set synchronously, but the way the ros clock gets notified involves a pub/sub that happens
  // AFTER the synchronous notification that the parameter was set.  This may also get fixed
  // upstream
  spin_until_ros_time_updated(clock, node, value);
}

TEST_F(TestTimeSource, detachUnattached) {
  rclcpp::TimeSource ts;

  ASSERT_NO_THROW(ts.detachNode());

  // Try multiple detach to see if error
  ASSERT_NO_THROW(ts.detachNode());
}

TEST_F(TestTimeSource, reattach) {
  rclcpp::TimeSource ts;
  // Try reattach
  ASSERT_NO_THROW(ts.attachNode(node));
  ASSERT_NO_THROW(ts.attachNode(node));
}

TEST_F(TestTimeSource, ROS_time_valid_attach_detach) {
  rclcpp::TimeSource ts;
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  EXPECT_FALSE(ros_clock->ros_time_is_active());
  ts.attachClock(ros_clock);
  auto now = ros_clock->now();
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  ts.attachNode(node);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  ts.detachNode();
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  ts.attachNode(node);
  EXPECT_FALSE(ros_clock->ros_time_is_active());
}

TEST_F(TestTimeSource, ROS_time_valid_wall_time) {
  rclcpp::TimeSource ts;
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto ros_clock2 = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  ts.attachNode(node);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  ts.attachClock(ros_clock2);
  EXPECT_FALSE(ros_clock2->ros_time_is_active());
}

TEST_F(TestTimeSource, ROS_time_valid_sim_time) {
  rclcpp::TimeSource ts;
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto ros_clock2 = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  set_use_sim_time_parameter(node, rclcpp::ParameterValue(true), ros_clock);
  ts.attachNode(node);
  EXPECT_TRUE(ros_clock->ros_time_is_active());

  ts.attachClock(ros_clock2);
  EXPECT_TRUE(ros_clock2->ros_time_is_active());
}

TEST_F(TestTimeSource, clock) {
  rclcpp::TimeSource ts(node);
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_FALSE(ros_clock->ros_time_is_active());
  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  trigger_clock_changes(node, ros_clock, false);

  // Even now that we've recieved a message, ROS time should still not be active since the
  // parameter has not been explicitly set.
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  // Activate ROS time.
  set_use_sim_time_parameter(node, rclcpp::ParameterValue(true), ros_clock);
  EXPECT_TRUE(ros_clock->ros_time_is_active());

  trigger_clock_changes(node, ros_clock);

  auto t_out = ros_clock->now();

  // Time from clock should now reflect what was published on the /clock topic.
  auto t_low = rclcpp::Time(1, 0, RCL_ROS_TIME);
  auto t_high = rclcpp::Time(10, 100000, RCL_ROS_TIME);
  EXPECT_NE(0L, t_out.nanoseconds());
  EXPECT_LT(t_low.nanoseconds(), t_out.nanoseconds());
  EXPECT_GT(t_high.nanoseconds(), t_out.nanoseconds());
}

class CallbackObject
{
public:
  int pre_callback_calls_ = 0;
  int last_precallback_id_ = 0;
  void pre_callback(int id)
  {
    last_precallback_id_ = id;
    ++pre_callback_calls_;
  }

  int post_callback_calls_ = 0;
  int last_postcallback_id_ = 0;
  rcl_time_jump_t last_timejump_;
  void post_callback(const rcl_time_jump_t & jump, int id)
  {
    last_postcallback_id_ = id; last_timejump_ = jump;
    ++post_callback_calls_;
  }
};

TEST_F(TestTimeSource, callbacks) {
  CallbackObject cbo;
  rcl_jump_threshold_t jump_threshold;
  jump_threshold.min_forward.nanoseconds = 0;
  jump_threshold.min_backward.nanoseconds = 0;
  jump_threshold.on_clock_change = true;

  rclcpp::TimeSource ts(node);
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // Register a callback for time jumps
  rclcpp::JumpHandler::SharedPtr callback_handler = ros_clock->create_jump_callback(
    std::bind(&CallbackObject::pre_callback, &cbo, 1),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 1),
    jump_threshold);

  EXPECT_EQ(0, cbo.last_precallback_id_);
  EXPECT_EQ(0, cbo.last_postcallback_id_);

  EXPECT_FALSE(ros_clock->ros_time_is_active());

  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  // Last arg below is 'expect_time_update'  Since ros_time is not active yet, we don't expect
  // the simulated time to be updated by trigger_clock_changes.  The method will pump messages
  // anyway, but won't fail the test when the simulated time doesn't update
  trigger_clock_changes(node, ros_clock, false);
  auto t_low = rclcpp::Time(1, 0, RCL_ROS_TIME);
  auto t_high = rclcpp::Time(10, 100000, RCL_ROS_TIME);

  // Callbacks will not be triggered since ROS time is not active.
  EXPECT_EQ(0, cbo.last_precallback_id_);
  EXPECT_EQ(0, cbo.last_postcallback_id_);

  // Activate ROS time.
  set_use_sim_time_parameter(node, rclcpp::ParameterValue(true), ros_clock);
  EXPECT_TRUE(ros_clock->ros_time_is_active());

  trigger_clock_changes(node, ros_clock);

  auto t_out = ros_clock->now();

  EXPECT_NE(0L, t_out.nanoseconds());
  EXPECT_LT(t_low.nanoseconds(), t_out.nanoseconds());
  EXPECT_GT(t_high.nanoseconds(), t_out.nanoseconds());

  // Callbacks will now have been triggered since ROS time is active.
  EXPECT_EQ(1, cbo.last_precallback_id_);
  EXPECT_EQ(1, cbo.last_postcallback_id_);

  // Change callbacks
  rclcpp::JumpHandler::SharedPtr callback_handler2 = ros_clock->create_jump_callback(
    std::bind(&CallbackObject::pre_callback, &cbo, 2),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 2),
    jump_threshold);

  trigger_clock_changes(node, ros_clock);

  EXPECT_EQ(2, cbo.last_precallback_id_);
  EXPECT_EQ(2, cbo.last_postcallback_id_);

  EXPECT_TRUE(ros_clock->ros_time_is_active());

  t_out = ros_clock->now();

  EXPECT_NE(0L, t_out.nanoseconds());
  EXPECT_LT(t_low.nanoseconds(), t_out.nanoseconds());
  EXPECT_GT(t_high.nanoseconds(), t_out.nanoseconds());

  // Register a callback handler with only pre_callback
  rclcpp::JumpHandler::SharedPtr callback_handler3 = ros_clock->create_jump_callback(
    std::bind(&CallbackObject::pre_callback, &cbo, 3),
    std::function<void(rcl_time_jump_t)>(),
    jump_threshold);

  trigger_clock_changes(node, ros_clock);
  EXPECT_EQ(3, cbo.last_precallback_id_);
  EXPECT_EQ(2, cbo.last_postcallback_id_);

  // Register a callback handler with only post_callback
  rclcpp::JumpHandler::SharedPtr callback_handler4 = ros_clock->create_jump_callback(
    std::function<void()>(),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 4),
    jump_threshold);

  trigger_clock_changes(node, ros_clock);
  EXPECT_EQ(3, cbo.last_precallback_id_);
  EXPECT_EQ(4, cbo.last_postcallback_id_);
}


TEST_F(TestTimeSource, callback_handler_erasure) {
  CallbackObject cbo;
  rcl_jump_threshold_t jump_threshold;
  jump_threshold.min_forward.nanoseconds = 0;
  jump_threshold.min_backward.nanoseconds = 0;
  jump_threshold.on_clock_change = true;

  rclcpp::TimeSource ts(node);
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  // Register a callback for time jumps
  rclcpp::JumpHandler::SharedPtr callback_handler = ros_clock->create_jump_callback(
    std::bind(&CallbackObject::pre_callback, &cbo, 1),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 1),
    jump_threshold);

  // Second callback handler
  rclcpp::JumpHandler::SharedPtr callback_handler2 = ros_clock->create_jump_callback(
    std::bind(&CallbackObject::pre_callback, &cbo, 1),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 1),
    jump_threshold);

  // Callbacks will not be triggered since ROS time is not active.
  EXPECT_EQ(0, cbo.last_precallback_id_);
  EXPECT_EQ(0, cbo.last_postcallback_id_);

  // Activate ROS time.
  set_use_sim_time_parameter(node, rclcpp::ParameterValue(true), ros_clock);
  EXPECT_TRUE(ros_clock->ros_time_is_active());

  trigger_clock_changes(node, ros_clock);

  auto t_low = rclcpp::Time(1, 0, RCL_ROS_TIME);
  auto t_high = rclcpp::Time(10, 100000, RCL_ROS_TIME);

  // Callbacks will now have been triggered since ROS time is active.
  EXPECT_EQ(1, cbo.last_precallback_id_);
  EXPECT_EQ(1, cbo.last_postcallback_id_);

  auto t_out = ros_clock->now();

  EXPECT_NE(0L, t_out.nanoseconds());
  EXPECT_LT(t_low.nanoseconds(), t_out.nanoseconds());
  EXPECT_GT(t_high.nanoseconds(), t_out.nanoseconds());

  // Requeue a pointer in a new position
  callback_handler = ros_clock->create_jump_callback(
    std::bind(&CallbackObject::pre_callback, &cbo, 2),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 2),
    jump_threshold);

  // Remove the last callback in the vector
  callback_handler2.reset();

  trigger_clock_changes(node, ros_clock);

  EXPECT_EQ(2, cbo.last_precallback_id_);
  EXPECT_EQ(2, cbo.last_postcallback_id_);

  EXPECT_TRUE(ros_clock->ros_time_is_active());

  t_out = ros_clock->now();

  EXPECT_NE(0L, t_out.nanoseconds());
  EXPECT_LT(t_low.nanoseconds(), t_out.nanoseconds());
  EXPECT_GT(t_high.nanoseconds(), t_out.nanoseconds());
}

TEST_F(TestTimeSource, parameter_activation) {
  rclcpp::TimeSource ts(node);
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  set_use_sim_time_parameter(node, rclcpp::ParameterValue(true), ros_clock);
  EXPECT_TRUE(ros_clock->ros_time_is_active());

  set_use_sim_time_parameter(node, rclcpp::ParameterValue(false), ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  // If the use_sim_time parameter is not explicitly set to True, this clock's use of sim time
  // should not be affected by the presence of a clock publisher.
  trigger_clock_changes(node, ros_clock, false);
  EXPECT_FALSE(ros_clock->ros_time_is_active());
  set_use_sim_time_parameter(node, rclcpp::ParameterValue(false), ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());
  set_use_sim_time_parameter(node, rclcpp::ParameterValue(true), ros_clock);
  EXPECT_TRUE(ros_clock->ros_time_is_active());
}

TEST_F(TestTimeSource, no_pre_jump_callback) {
  CallbackObject cbo;
  rcl_jump_threshold_t jump_threshold;
  jump_threshold.min_forward.nanoseconds = 0;
  jump_threshold.min_backward.nanoseconds = 0;
  jump_threshold.on_clock_change = true;

  rclcpp::TimeSource ts(node);
  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // Register a callback for time jumps
  rclcpp::JumpHandler::SharedPtr callback_handler = ros_clock->create_jump_callback(
    nullptr,
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 1),
    jump_threshold);

  ASSERT_EQ(0, cbo.last_precallback_id_);
  ASSERT_EQ(0, cbo.last_postcallback_id_);
  ts.attachClock(ros_clock);

  // Activate ROS time
  set_use_sim_time_parameter(node, rclcpp::ParameterValue(true), ros_clock);
  ASSERT_TRUE(ros_clock->ros_time_is_active());

  EXPECT_EQ(0, cbo.last_precallback_id_);
  EXPECT_EQ(0, cbo.pre_callback_calls_);
  EXPECT_EQ(1, cbo.last_postcallback_id_);
  EXPECT_EQ(1, cbo.post_callback_calls_);
}
