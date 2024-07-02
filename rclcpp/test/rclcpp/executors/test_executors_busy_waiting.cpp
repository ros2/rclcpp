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
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "./executor_types.hpp"
#include "./test_waitable.hpp"

using namespace std::chrono_literals;

template<typename T>
class TestBusyWaiting : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    const auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::stringstream test_name;
    test_name << test_info->test_case_name() << "_" << test_info->name();
    node = std::make_shared<rclcpp::Node>("node", test_name.str());
    callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      /* automatically_add_to_executor_with_node =*/ false);

    auto waitable_interfaces = node->get_node_waitables_interface();
    waitable = std::make_shared<TestWaitable>();
    waitable_interfaces->add_waitable(waitable, callback_group);

    executor = std::make_shared<T>();
    executor->add_callback_group(callback_group, node->get_node_base_interface());
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  void
  set_up_and_trigger_waitable(std::function<void()> extra_callback = nullptr)
  {
    this->has_executed = false;
    this->waitable->set_on_execute_callback([this, extra_callback]() {
        if (!this->has_executed) {
        // trigger once to see if the second trigger is handled or not
        // this follow up trigger simulates new entities becoming ready while
        // the executor is executing something else, e.g. subscription got data
        // or a timer expired, etc.
        // spin_some would not handle this second trigger, since it collects
        // work only once, whereas spin_all should handle it since it
        // collects work multiple times
          this->waitable->trigger();
          this->has_executed = true;
        }
        if (nullptr != extra_callback) {
          extra_callback();
        }
    });
    this->waitable->trigger();
  }

  void
  check_for_busy_waits(std::chrono::steady_clock::time_point start_time)
  {
    // rough time based check, since the work to be done was very small it
    // should be safe to check that we didn't use more than half the
    // max duration, which itself is much larger than necessary
    // however, it could still produce a false-positive
    EXPECT_LT(
      std::chrono::steady_clock::now() - start_time,
      max_duration / 2)
      << "executor took a long time to execute when it should have done "
      << "nothing and should not have blocked either, but this could be a "
      << "false negative if the computer is really slow";

    // this check is making some assumptions about the implementation of the
    // executors, but it should be safe to say that a busy wait may result in
    // hundreds or thousands of calls to is_ready(), but "normal" executor
    // behavior should be within an order of magnitude of the number of
    // times that the waitable was executed
    ASSERT_LT(waitable->get_is_ready_call_count(), 10u * this->waitable->get_count());
  }

  static constexpr auto max_duration = 10s;

  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  std::shared_ptr<TestWaitable> waitable;
  std::chrono::steady_clock::time_point start_time;
  std::shared_ptr<T> executor;
  bool has_executed;
};

TYPED_TEST_SUITE(TestBusyWaiting, ExecutorTypes, ExecutorTypeNames);

TYPED_TEST(TestBusyWaiting, test_spin_all)
{
  this->set_up_and_trigger_waitable();

  auto start_time = std::chrono::steady_clock::now();
  this->executor->spin_all(this->max_duration);
  this->check_for_busy_waits(start_time);
  // this should get the initial trigger, and the follow up from in the callback
  ASSERT_EQ(this->waitable->get_count(), 2u);
}

TYPED_TEST(TestBusyWaiting, test_spin_some)
{
  this->set_up_and_trigger_waitable();

  auto start_time = std::chrono::steady_clock::now();
  this->executor->spin_some(this->max_duration);
  this->check_for_busy_waits(start_time);
  // this should get the inital trigger, but not the follow up in the callback
  ASSERT_EQ(this->waitable->get_count(), 1u);
}

TYPED_TEST(TestBusyWaiting, test_spin)
{
  std::condition_variable cv;
  std::mutex cv_m;
  bool first_check_passed = false;

  this->set_up_and_trigger_waitable([&cv, &cv_m, &first_check_passed]() {
      cv.notify_one();
      if (!first_check_passed) {
        std::unique_lock<std::mutex> lk(cv_m);
        cv.wait_for(lk, 1s, [&]() {return first_check_passed;});
      }
  });

  auto start_time = std::chrono::steady_clock::now();
  std::thread t([this]() {
      this->executor->spin();
    });

  // wait until thread has started (first execute of waitable)
  {
    std::unique_lock<std::mutex> lk(cv_m);
    cv.wait_for(lk, 10s);
  }
  EXPECT_GT(this->waitable->get_count(), 0u);

  first_check_passed = true;
  cv.notify_one();

  // wait until the executor has finished (second execute of waitable)
  {
    std::unique_lock<std::mutex> lk(cv_m);
    cv.wait_for(lk, 10s);
  }
  EXPECT_EQ(this->waitable->get_count(), 2u);

  this->executor->cancel();
  t.join();

  this->check_for_busy_waits(start_time);
  // this should get the initial trigger, and the follow up from in the callback
  ASSERT_EQ(this->waitable->get_count(), 2u);
}
