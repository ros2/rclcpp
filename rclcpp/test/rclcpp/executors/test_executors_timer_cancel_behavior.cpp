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
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/utilities.hpp"

#include "rosgraph_msgs/msg/clock.hpp"

#include "./executor_types.hpp"

using namespace std::chrono_literals;

class TimerNode : public rclcpp::Node
{
public:
  explicit TimerNode(std::string subname)
  : Node("timer_node", subname)
  {
  }

  void CreateTimer1()
  {
    timer1_ = rclcpp::create_timer(
      this->get_node_base_interface(), get_node_timers_interface(),
      get_clock(), 1ms,
      std::bind(&TimerNode::Timer1Callback, this));
  }

  void CreateTimer2()
  {
    timer2_ = rclcpp::create_timer(
      this->get_node_base_interface(), get_node_timers_interface(),
      get_clock(), 1ms,
      std::bind(&TimerNode::Timer2Callback, this));
  }

  int GetTimer1Cnt()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return cnt1_;
  }
  int GetTimer2Cnt()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return cnt2_;
  }

  void ResetTimer1()
  {
    timer1_->reset();
  }

  void ResetTimer2()
  {
    timer2_->reset();
  }

  void CancelTimer1()
  {
    RCLCPP_DEBUG(this->get_logger(), "Timer 1 cancelling!");
    timer1_->cancel();
  }

  void CancelTimer2()
  {
    RCLCPP_DEBUG(this->get_logger(), "Timer 2 cancelling!");
    timer2_->cancel();
  }

private:
  void Timer1Callback()
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      cnt1_++;
    }
    RCLCPP_DEBUG(this->get_logger(), "Timer 1! (%d)", cnt1_);
  }

  void Timer2Callback()
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      cnt2_++;
    }
    RCLCPP_DEBUG(this->get_logger(), "Timer 2! (%d)", cnt2_);
  }

  std::mutex mutex_;

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  int cnt1_ = 0;
  int cnt2_ = 0;
};

// Sets up a separate thread to publish /clock messages.
// Clock rate relative to real clock is controlled by realtime_update_rate.
// This is set conservatively slow to ensure unit tests are reliable on Windows
// environments, where timing performance is subpar.
//
// Use `sleep_for` in tests to advance the clock. Clock should run and be published
// in separate thread continuously to ensure correct behavior in node under test.
class ClockPublisher : public rclcpp::Node
{
public:
  explicit ClockPublisher(float simulated_clock_step = .001f, float realtime_update_rate = 0.25f)
  : Node("clock_publisher"),
    ros_update_duration_(0, 0),
    realtime_clock_step_(0, 0),
    rostime_(0, 0)
  {
    clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
    realtime_clock_step_ =
      rclcpp::Duration::from_seconds(simulated_clock_step / realtime_update_rate);
    ros_update_duration_ = rclcpp::Duration::from_seconds(simulated_clock_step);

    timer_thread_ = std::thread(&ClockPublisher::RunTimer, this);
  }

  ~ClockPublisher()
  {
    running_ = false;
    if (timer_thread_.joinable()) {
      timer_thread_.join();
    }
  }

  bool wait_for_connection(std::chrono::nanoseconds timeout)
  {
    auto end_time = std::chrono::steady_clock::now() + timeout;
    while (clock_publisher_->get_subscription_count() == 0 &&
      (std::chrono::steady_clock::now() < end_time))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return clock_publisher_->get_subscription_count() != 0;
  }

  void sleep_for(rclcpp::Duration duration)
  {
    rclcpp::Time start_time(0, 0, RCL_ROS_TIME);
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      start_time = rostime_;
    }
    rclcpp::Time current_time = start_time;

    while (true) {
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        current_time = rostime_;
      }
      if ((current_time - start_time) >= duration) {
        return;
      }
      std::this_thread::sleep_for(realtime_clock_step_.to_chrono<std::chrono::milliseconds>());
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        rostime_ += ros_update_duration_;
      }
    }
  }

private:
  void RunTimer()
  {
    while (running_) {
      PublishClock();
      std::this_thread::sleep_for(realtime_clock_step_.to_chrono<std::chrono::milliseconds>());
    }
  }

  void PublishClock()
  {
    auto message = rosgraph_msgs::msg::Clock();
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      message.clock = rostime_;
    }
    clock_publisher_->publish(message);
  }

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;

  rclcpp::Duration ros_update_duration_;
  rclcpp::Duration realtime_clock_step_;
  // Rostime must be guarded by a mutex, since accessible in running thread
  // as well as sleep_for
  rclcpp::Time rostime_;
  std::mutex mutex_;
  std::thread timer_thread_;
  std::atomic<bool> running_ = true;
};


template<typename T>
class TestTimerCancelBehavior : public ::testing::Test
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

  void SetUp()
  {
    const auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::stringstream test_name;
    test_name << test_info->test_case_name() << "_" << test_info->name();
    node = std::make_shared<TimerNode>(test_name.str());
    param_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    ASSERT_TRUE(param_client->wait_for_service(5s));

    auto set_parameters_results = param_client->set_parameters(
      {rclcpp::Parameter("use_sim_time", true)});
    for (auto & result : set_parameters_results) {
      ASSERT_TRUE(result.successful);
    }

    // Check if the clock type is simulation time
    EXPECT_EQ(RCL_ROS_TIME, node->get_clock()->get_clock_type());

    // Create timers
    this->node->CreateTimer1();
    this->node->CreateTimer2();

    // Run standalone thread to publish clock time
    sim_clock_node = std::make_shared<ClockPublisher>();

    // Spin the executor in a standalone thread
    executor.add_node(this->node);
    standalone_thread = std::thread(
      [this]() {
        executor.spin();
      });

    EXPECT_TRUE(this->sim_clock_node->wait_for_connection(50ms));
    EXPECT_EQ(RCL_ROS_TIME, node->get_clock()->ros_time_is_active());
  }

  void TearDown()
  {
    executor.cancel();

    // Clean up thread object
    if (standalone_thread.joinable()) {
      standalone_thread.join();
    }

    node.reset();
    sim_clock_node.reset();
  }

  std::shared_ptr<TimerNode> node;
  std::shared_ptr<ClockPublisher> sim_clock_node;
  rclcpp::SyncParametersClient::SharedPtr param_client;
  std::thread standalone_thread;
  T executor;
};

using MainExecutorTypes =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::executors::StaticSingleThreadedExecutor>;

// TODO(@fujitatomoya): this test excludes EventExecutor because it does not
// support simulation time used for this test to relax the racy condition.
// See more details for https://github.com/ros2/rclcpp/issues/2457.
TYPED_TEST_SUITE(TestTimerCancelBehavior, MainExecutorTypes, ExecutorTypeNames);

TYPED_TEST(TestTimerCancelBehavior, testTimer1CancelledWithExecutorSpin) {
  // Validate that cancelling one timer yields no change in behavior for other
  // timers. Specifically, this tests the behavior when using spin() to run the
  // executor, which is the most common usecase.

  // Cancel to stop the spin after some time.
  this->sim_clock_node->sleep_for(50ms);
  this->node->CancelTimer1();
  this->sim_clock_node->sleep_for(150ms);

  int t1_runs = this->node->GetTimer1Cnt();
  int t2_runs = this->node->GetTimer2Cnt();
  EXPECT_NE(t1_runs, t2_runs);
  // Check that t2 has significantly more calls
  EXPECT_LT(t1_runs + 50, t2_runs);
}

TYPED_TEST(TestTimerCancelBehavior, testTimer2CancelledWithExecutorSpin) {
  // Validate that cancelling one timer yields no change in behavior for other
  // timers. Specifically, this tests the behavior when using spin() to run the
  // executor, which is the most common usecase.

  // Cancel to stop the spin after some time.
  this->sim_clock_node->sleep_for(50ms);
  this->node->CancelTimer2();
  this->sim_clock_node->sleep_for(150ms);

  int t1_runs = this->node->GetTimer1Cnt();
  int t2_runs = this->node->GetTimer2Cnt();
  EXPECT_NE(t1_runs, t2_runs);
  // Check that t1 has significantly more calls
  EXPECT_LT(t2_runs + 50, t1_runs);
}

TYPED_TEST(TestTimerCancelBehavior, testHeadTimerCancelThenResetBehavior) {
  // Validate that cancelling timer doesn't affect operation of other timers,
  // and that the cancelled timer starts executing normally once reset manually.

  // Cancel to stop the spin after some time.
  this->sim_clock_node->sleep_for(50ms);
  this->node->CancelTimer1();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_initial = this->node->GetTimer1Cnt();
  int t2_runs_initial = this->node->GetTimer2Cnt();

  // Manually reset timer 1, then sleep again
  // Counts should update.
  this->node->ResetTimer1();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_final = this->node->GetTimer1Cnt();
  int t2_runs_final = this->node->GetTimer2Cnt();

  // T1 should have been restarted, and execute about 15 additional times.
  // Check 10 greater than initial, to account for some timing jitter.
  EXPECT_LT(t1_runs_initial + 50, t1_runs_final);

  EXPECT_LT(t1_runs_initial + 50, t2_runs_initial);
  // Check that t2 has significantly more calls, and keeps getting called.
  EXPECT_LT(t2_runs_initial + 50, t2_runs_final);
}

TYPED_TEST(TestTimerCancelBehavior, testBackTimerCancelThenResetBehavior) {
  // Validate that cancelling timer doesn't affect operation of other timers,
  // and that the cancelled timer starts executing normally once reset manually.

  // Cancel to stop the spin after some time.
  this->sim_clock_node->sleep_for(50ms);
  this->node->CancelTimer2();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_initial = this->node->GetTimer1Cnt();
  int t2_runs_initial = this->node->GetTimer2Cnt();

  // Manually reset timer 1, then sleep again
  // Counts should update.
  this->node->ResetTimer2();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_final = this->node->GetTimer1Cnt();
  int t2_runs_final = this->node->GetTimer2Cnt();

  // T2 should have been restarted, and execute about 15 additional times.
  // Check 10 greater than initial, to account for some timing jitter.
  EXPECT_LT(t2_runs_initial + 50, t2_runs_final);

  EXPECT_LT(t2_runs_initial + 50, t1_runs_initial);
  // Check that t1 has significantly more calls, and keeps getting called.
  EXPECT_LT(t1_runs_initial + 50, t1_runs_final);
}

TYPED_TEST(TestTimerCancelBehavior, testBothTimerCancelThenResetT1Behavior) {
  // Validate behavior from cancelling 2 timers, then only re-enabling one of them.
  // Ensure that only the reset timer is executed.

  // Cancel to stop the spin after some time.
  this->sim_clock_node->sleep_for(50ms);
  this->node->CancelTimer1();
  this->node->CancelTimer2();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_initial = this->node->GetTimer1Cnt();
  int t2_runs_initial = this->node->GetTimer2Cnt();

  // Manually reset timer 1, then sleep again
  // Counts should update.
  this->node->ResetTimer1();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_intermediate = this->node->GetTimer1Cnt();
  int t2_runs_intermediate = this->node->GetTimer2Cnt();

  this->node->ResetTimer2();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_final = this->node->GetTimer1Cnt();
  int t2_runs_final = this->node->GetTimer2Cnt();

  // T1 and T2 should have the same initial count.
  EXPECT_LE(std::abs(t1_runs_initial - t2_runs_initial), 1);

  // Expect that T1 has up to 15 more calls than t2. Add some buffer
  // to account for jitter.
  EXPECT_EQ(t2_runs_initial, t2_runs_intermediate);
  EXPECT_LT(t1_runs_initial + 50, t1_runs_intermediate);

  // Expect that by end of test, both are running properly again.
  EXPECT_LT(t1_runs_intermediate + 50, t1_runs_final);
  EXPECT_LT(t2_runs_intermediate + 50, t2_runs_final);
}

TYPED_TEST(TestTimerCancelBehavior, testBothTimerCancelThenResetT2Behavior) {
  // Validate behavior from cancelling 2 timers, then only re-enabling one of them.
  // Ensure that only the reset timer is executed.

  // Cancel to stop the spin after some time.
  this->sim_clock_node->sleep_for(50ms);
  this->node->CancelTimer1();
  this->node->CancelTimer2();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_initial = this->node->GetTimer1Cnt();
  int t2_runs_initial = this->node->GetTimer2Cnt();

  // Manually reset timer 1, then sleep again
  // Counts should update.
  this->node->ResetTimer2();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_intermediate = this->node->GetTimer1Cnt();
  int t2_runs_intermediate = this->node->GetTimer2Cnt();

  this->node->ResetTimer1();
  this->sim_clock_node->sleep_for(150ms);
  int t1_runs_final = this->node->GetTimer1Cnt();
  int t2_runs_final = this->node->GetTimer2Cnt();

  // T1 and T2 should have the same initial count.
  EXPECT_LE(std::abs(t1_runs_initial - t2_runs_initial), 1);

  // Expect that T1 has up to 15 more calls than t2. Add some buffer
  // to account for jitter.
  EXPECT_EQ(t1_runs_initial, t1_runs_intermediate);
  EXPECT_LT(t2_runs_initial + 50, t2_runs_intermediate);

  // Expect that by end of test, both are running properly again.
  EXPECT_LT(t1_runs_intermediate + 50, t1_runs_final);
  EXPECT_LT(t2_runs_intermediate + 50, t2_runs_final);
}
