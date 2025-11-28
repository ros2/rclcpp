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

#ifndef RCLCPP__WAITABLES__WAITABLE_TEST_HELPERS_HPP_
#define RCLCPP__WAITABLES__WAITABLE_TEST_HELPERS_HPP_

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/wait_set.hpp>

namespace rclcpp
{
namespace test
{
namespace waitables
{

/// Test that a given waitable is ready after a second wait.
/**
 * The purpose of this test is to check that a waitable will remain ready
 * on subsequent wait calls, if that is the expected behavior.
 * Not all waitables should remain ready after a wait call, which can be
 * expressed in the expected_to_stay_ready argument which defaults to true.
 * If set to false, it will check that it is not ready after a second wait, as
 * well as some other parts of the test.
 *
 * The given waitable should:
 *
 *   - not be ready initially
 *   - not be ready after being waited on (and timing out)
 *   - should become ready after the make_waitable_ready method is called
 *   - may or may not be ready at this point
 *   - should be ready after waiting on it, within the wait_timeout
 *   - should be ready still after a second wait (unless expected_to_stay_ready = false)
 *   - if expected_to_stay_ready, should become not ready after a take_data/execute
 */
template<typename WaitableT>
void
do_test_that_waitable_stays_ready_after_second_wait(
  const std::shared_ptr<WaitableT> & waitable,
  std::function<void()> make_waitable_ready,
  bool expected_to_stay_ready = true,
  std::chrono::nanoseconds wait_timeout = std::chrono::seconds(5))
{
  rclcpp::WaitSet wait_set;
  wait_set.add_waitable(waitable);

  // not ready initially
  EXPECT_FALSE(waitable->is_ready(wait_set.get_rcl_wait_set()))
    << "waitable is unexpectedly ready before waiting";

  // not ready after a wait that timesout
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(0));
    EXPECT_EQ(wait_result.kind(), rclcpp::WaitResultKind::Timeout)
      << "wait set did not timeout as expected";
    EXPECT_FALSE(waitable->is_ready(wait_set.get_rcl_wait_set()))
      << "waitable is unexpectedly ready after waiting, but before making ready";
  }

  // make it ready and wait on it
  make_waitable_ready();
  {
    auto wait_result = wait_set.wait(wait_timeout);
    EXPECT_EQ(wait_result.kind(), rclcpp::WaitResultKind::Ready)
      << "wait set was not ready after the waitable should have been made ready";
    EXPECT_TRUE(waitable->is_ready(wait_set.get_rcl_wait_set()))
      << "waitable is unexpectedly not ready after making it ready and waiting";
  }

  // wait again, and see that it is ready as expected or not expected
  {
    auto wait_result = wait_set.wait(std::chrono::seconds(0));
    if (expected_to_stay_ready) {
      EXPECT_EQ(wait_result.kind(), rclcpp::WaitResultKind::Ready)
        << "wait set was not ready on a second wait on the waitable";
      EXPECT_TRUE(waitable->is_ready(wait_set.get_rcl_wait_set()))
        << "waitable unexpectedly not ready after second wait";
    } else {
      EXPECT_EQ(wait_result.kind(), rclcpp::WaitResultKind::Timeout)
        << "wait set did not time out after the waitable should have no longer been ready";
      EXPECT_FALSE(waitable->is_ready(wait_set.get_rcl_wait_set()))
        << "waitable was ready after waiting a second time, which was not expected";
    }
  }

  // if expected_to_stay_ready, check that take_data/execute makes it not ready
  if (expected_to_stay_ready) {
    waitable->execute(waitable->take_data());
    auto wait_result = wait_set.wait(std::chrono::seconds(0));
    EXPECT_EQ(wait_result.kind(), rclcpp::WaitResultKind::Timeout)
      << "wait set did not time out after the waitable should have no longer been ready";
    EXPECT_FALSE(waitable->is_ready(wait_set.get_rcl_wait_set()))
      << "waitable was unexpectedly ready after a take_data and execute";
  }
}

}  // namespace waitables
}  // namespace test
}  // namespace rclcpp

#endif  // RCLCPP__WAITABLES__WAITABLE_TEST_HELPERS_HPP_
