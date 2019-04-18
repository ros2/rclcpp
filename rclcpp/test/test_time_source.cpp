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

static const std::shared_future<void> g_no_future = {};
static constexpr bool g_do_not_wait_for_matched = false;
static constexpr int g_number_of_clock_messages = 5;
void
trigger_clock_changes(
  rclcpp::Node::SharedPtr node,
  std::shared_future<void> post_future,
  bool wait_for_matched = true)
{
  auto clock_pub =
    node->create_publisher<rosgraph_msgs::msg::Clock>("clock", rmw_qos_profile_default);

  // wait for matching subscription
  if (wait_for_matched) {
    auto start = std::chrono::steady_clock::now();
    do {
      if (clock_pub->get_subscription_count() > 0u) {
        break;
      }
      std::this_thread::sleep_for(10ms);  // just to avoid full busy wait
    } while (std::chrono::steady_clock::now() - start <= 10s);
    EXPECT_NE(0u, clock_pub->get_subscription_count());
  }

  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::WallRate loop_rate(50);
    for (int i = 0; i < g_number_of_clock_messages; ++i) {
      if (!rclcpp::ok()) {
        break;  // Break for ctrl-c
      }
      auto msg = std::make_shared<rosgraph_msgs::msg::Clock>();
      msg->clock.sec = i;
      msg->clock.nanosec = 1000;
      clock_pub->publish(msg);
      executor.spin_once(1ms);
      loop_rate.sleep();
    }
  }

  if (post_future.valid()) {
    // wait for at least one post callback, or 10s to pass
    EXPECT_EQ(
      rclcpp::spin_until_future_complete(node, post_future, 10s),
      rclcpp::executor::FutureReturnCode::SUCCESS);
  }
}

void
set_and_check_use_sim_time_parameter(
  rclcpp::Node::SharedPtr node,
  rclcpp::ParameterValue value,
  const bool expected)
{
  // set the parameter
  node->set_parameter({"use_sim_time", value});
  // then spin until it has been updated or some time has passed
  rclcpp::Parameter use_sim_time;
  auto timeout = 10s;
  auto wait_period = 100ms;
  auto start = std::chrono::steady_clock::now();
  rclcpp::executors::SingleThreadedExecutor executor;
  do {
    if (
      node->get_parameter("use_sim_time", use_sim_time) &&
      use_sim_time.get_parameter_value() == value &&
      expected == node->get_clock()->ros_time_is_active())
    {
      break;
    }
    executor.spin_node_once(node, wait_period);
  } while (std::chrono::steady_clock::now() <= start + timeout);
  EXPECT_EQ(use_sim_time.get_parameter_value(), value);
  EXPECT_EQ(node->get_clock()->ros_time_is_active(), expected);
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
  ros_clock->now();
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

  node->set_parameter({"use_sim_time", true});
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

  trigger_clock_changes(node, g_no_future, g_do_not_wait_for_matched);

  // Even now that we've recieved a message, ROS time should still not be active since the
  // parameter has not been explicitly set.
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  // Activate ROS time.
  set_and_check_use_sim_time_parameter(node, rclcpp::ParameterValue(true), true);

  auto t_low = rclcpp::Time(1, 0, RCL_ROS_TIME);
  auto t_high = rclcpp::Time(10, 100000, RCL_ROS_TIME);

  trigger_clock_changes(node, g_no_future);
  // Time from clock should now (eventually) reflect what was published on the /clock topic.
  auto t_out = ros_clock->now();
  // If the clock has not been updated yet, then manually wait for this
  // condition to become true for a while.
  // We have to poll like this since we don't have the post jump callback in
  // which we can set a future and know for sure that a clock msg has been
  // received and therefore the time has been updated.
  {
    auto start = std::chrono::steady_clock::now();
    rclcpp::executors::SingleThreadedExecutor executor;
    do {
      if (0L < t_out.nanoseconds()) {
        break;
      }
      if (std::chrono::steady_clock::now() - start >= 5s) {
        // after 5s, try sending the data again...
        trigger_clock_changes(node, g_no_future);
      }
      executor.spin_node_once(node, 10ms);  // spin to let clock messages get processed
      t_out = ros_clock->now();  // check the time again
    } while (rclcpp::ok() && std::chrono::steady_clock::now() - start <= 10s);
  }
  EXPECT_NE(0L, t_out.nanoseconds());
  EXPECT_LT(t_low.nanoseconds(), t_out.nanoseconds());
  EXPECT_GT(t_high.nanoseconds(), t_out.nanoseconds());
}

class CallbackObject
{
public:
  std::shared_future<void>
  get_new_post_shared_future(
    int number_of_post_calls_before_completing_future = g_number_of_clock_messages)
  {
    std::lock_guard<std::mutex> lock(promise_mutex_);
    post_promise_has_been_set_ = false;
    number_of_post_calls_before_completing_future_ = number_of_post_calls_before_completing_future;
    post_promise_ = {};
    return post_promise_.get_future();
  }

  int pre_callback_calls_ = 0;
  int last_precallback_id_ = 0;

  void
  pre_callback(int id)
  {
    last_precallback_id_ = id;
    ++pre_callback_calls_;
  }

  int post_callback_calls_ = 0;
  int last_postcallback_id_ = 0;
  rcl_time_jump_t last_timejump_;

  void
  post_callback(const rcl_time_jump_t & jump, int id)
  {
    std::lock_guard<std::mutex> lock(promise_mutex_);
    last_postcallback_id_ = id;
    last_timejump_ = jump;
    ++post_callback_calls_;
    if (
      !post_promise_has_been_set_ &&
      post_callback_calls_ >= number_of_post_calls_before_completing_future_)
    {
      post_promise_.set_value();
      post_promise_has_been_set_ = true;
    }
  }

private:
  std::mutex promise_mutex_;
  bool post_promise_has_been_set_ = false;
  int number_of_post_calls_before_completing_future_ = 1;
  std::promise<void> post_promise_;
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
  (void)callback_handler;  // silence cppcheck about unread variable (weird because it's RAII)

  EXPECT_EQ(0, cbo.last_precallback_id_);
  EXPECT_EQ(0, cbo.last_postcallback_id_);

  EXPECT_FALSE(ros_clock->ros_time_is_active());

  ts.attachClock(ros_clock);
  EXPECT_FALSE(ros_clock->ros_time_is_active());

  trigger_clock_changes(node, g_no_future, g_do_not_wait_for_matched);
  auto t_low = rclcpp::Time(1, 0, RCL_ROS_TIME);
  auto t_high = rclcpp::Time(10, 100000, RCL_ROS_TIME);

  // Callbacks will not be triggered since ROS time is not active.
  EXPECT_EQ(0, cbo.last_precallback_id_);
  EXPECT_EQ(0, cbo.last_postcallback_id_);

  // Activate ROS time.
  set_and_check_use_sim_time_parameter(node, rclcpp::ParameterValue(true), true);

  trigger_clock_changes(node, cbo.get_new_post_shared_future());

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
  (void)callback_handler2;  // silence cppcheck about unread variable (weird because it's RAII)

  trigger_clock_changes(node, cbo.get_new_post_shared_future());

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
  (void)callback_handler3;  // silence cppcheck about unread variable (weird because it's RAII)

  trigger_clock_changes(node, cbo.get_new_post_shared_future());
  EXPECT_EQ(3, cbo.last_precallback_id_);
  EXPECT_EQ(2, cbo.last_postcallback_id_);

  // Register a callback handler with only post_callback
  rclcpp::JumpHandler::SharedPtr callback_handler4 = ros_clock->create_jump_callback(
    std::function<void()>(),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 4),
    jump_threshold);
  (void)callback_handler4;  // silence cppcheck about unread variable (weird because it's RAII)

  trigger_clock_changes(node, cbo.get_new_post_shared_future());
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
  (void)callback_handler;  // silence cppcheck about unread variable (weird because it's RAII)

  // Second callback handler
  rclcpp::JumpHandler::SharedPtr callback_handler2 = ros_clock->create_jump_callback(
    std::bind(&CallbackObject::pre_callback, &cbo, 1),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 1),
    jump_threshold);

  // Callbacks will not be triggered since ROS time is not active.
  EXPECT_EQ(0, cbo.last_precallback_id_);
  EXPECT_EQ(0, cbo.last_postcallback_id_);

  // Activate ROS time.
  set_and_check_use_sim_time_parameter(node, rclcpp::ParameterValue(true), true);

  trigger_clock_changes(node, cbo.get_new_post_shared_future());

  auto t_low = rclcpp::Time(1, 0, RCL_ROS_TIME);
  auto t_high = rclcpp::Time(10, 100000, RCL_ROS_TIME);

  // Callbacks will now have been triggered since ROS time is active.
  EXPECT_EQ(1, cbo.last_precallback_id_);
  EXPECT_EQ(1, cbo.last_postcallback_id_);
  EXPECT_TRUE(ros_clock->ros_time_is_active());

  auto t_out = ros_clock->now();

  EXPECT_NE(0L, t_out.nanoseconds());
  EXPECT_LT(t_low.nanoseconds(), t_out.nanoseconds());
  EXPECT_GT(t_high.nanoseconds(), t_out.nanoseconds());

  // Requeue a pointer in a new position
  callback_handler = ros_clock->create_jump_callback(
    std::bind(&CallbackObject::pre_callback, &cbo, 2),
    std::bind(&CallbackObject::post_callback, &cbo, std::placeholders::_1, 2),
    jump_threshold);
  (void)callback_handler;  // silence cppcheck about unread variable (weird because it's RAII)

  // Remove the last callback in the vector
  callback_handler2.reset();

  trigger_clock_changes(node, cbo.get_new_post_shared_future());

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

  set_and_check_use_sim_time_parameter(node, rclcpp::ParameterValue(true), true);

  set_and_check_use_sim_time_parameter(node, rclcpp::ParameterValue(false), false);

  // If the use_sim_time parameter is not explicitly set to True, this clock's use of sim time
  // should not be affected by the presence of a clock publisher.
  trigger_clock_changes(node, g_no_future, g_do_not_wait_for_matched);
  EXPECT_FALSE(ros_clock->ros_time_is_active());
  set_and_check_use_sim_time_parameter(node, rclcpp::ParameterValue(false), false);
  set_and_check_use_sim_time_parameter(node, rclcpp::ParameterValue(true), true);
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
  (void)callback_handler;  // silence cppcheck about unread variable (weird because it's RAII)

  ASSERT_EQ(0, cbo.last_precallback_id_);
  ASSERT_EQ(0, cbo.last_postcallback_id_);
  ts.attachClock(ros_clock);

  // Activate ROS time
  set_and_check_use_sim_time_parameter(node, rclcpp::ParameterValue(true), true);

  EXPECT_EQ(0, cbo.last_precallback_id_);
  EXPECT_EQ(0, cbo.pre_callback_calls_);
  auto future = cbo.get_new_post_shared_future(1);
  EXPECT_EQ(
    rclcpp::spin_until_future_complete(node, future, 10s),
    rclcpp::executor::FutureReturnCode::SUCCESS);
  EXPECT_EQ(1, cbo.last_postcallback_id_);
  EXPECT_EQ(1, cbo.post_callback_calls_);
}
