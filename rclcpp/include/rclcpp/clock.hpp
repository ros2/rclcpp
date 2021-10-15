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

#ifndef RCLCPP__CLOCK_HPP_
#define RCLCPP__CLOCK_HPP_

#include <functional>
#include <memory>
#include <mutex>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl/time.h"
#include "rcutils/time.h"
#include "rcutils/types/rcutils_ret.h"

namespace rclcpp
{

class TimeSource;

class JumpHandler
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(JumpHandler)

  using pre_callback_t = std::function<void ()>;
  using post_callback_t = std::function<void (const rcl_time_jump_t &)>;

  JumpHandler(
    pre_callback_t pre_callback,
    post_callback_t post_callback,
    const rcl_jump_threshold_t & threshold);

  pre_callback_t pre_callback;
  post_callback_t post_callback;
  rcl_jump_threshold_t notice_threshold;
};

class Clock
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Clock)

  /// Default c'tor
  /**
   * Initializes the clock instance with the given clock_type.
   *
   * \param clock_type type of the clock.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  explicit Clock(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  RCLCPP_PUBLIC
  ~Clock();

  /**
   * Returns current time from the time source specified by clock_type.
   *
   * \return current time.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   */
  RCLCPP_PUBLIC
  Time
  now();

  /**
   * Returns the clock of the type `RCL_ROS_TIME` is active.
   *
   * \return true if the clock is active
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw if
   * the current clock does not have the clock_type `RCL_ROS_TIME`.
   */
  RCLCPP_PUBLIC
  bool
  ros_time_is_active();

  /// Return the rcl_clock_t clock handle
  RCLCPP_PUBLIC
  rcl_clock_t *
  get_clock_handle() noexcept;

  RCLCPP_PUBLIC
  rcl_clock_type_t
  get_clock_type() const noexcept;

  /// Get the clock's mutex
  RCLCPP_PUBLIC
  std::mutex &
  get_clock_mutex() noexcept;

  // Add a callback to invoke if the jump threshold is exceeded.
  /**
   * These callback functions must remain valid as long as the
   * returned shared pointer is valid.
   *
   * Function will register callbacks to the callback queue. On time jump all
   * callbacks will be executed whose threshold is greater then the time jump;
   * The logic will first call selected pre_callbacks and then all selected
   * post_callbacks.
   *
   * Function is only applicable if the clock_type is `RCL_ROS_TIME`
   *
   * \param pre_callback Must be non-throwing
   * \param post_callback Must be non-throwing.
   * \param threshold Callbacks will be triggered if the time jump is greater
   * then the threshold.
   * \throws anything rclcpp::exceptions::throw_from_rcl_error can throw.
   * \throws std::bad_alloc if the allocation of the JumpHandler fails.
   * \warning the instance of the clock must remain valid as long as any created
   * JumpHandler.
   */
  RCLCPP_PUBLIC
  JumpHandler::SharedPtr
  create_jump_callback(
    JumpHandler::pre_callback_t pre_callback,
    JumpHandler::post_callback_t post_callback,
    const rcl_jump_threshold_t & threshold);

private:
  // Invoke time jump callback
  RCLCPP_PUBLIC
  static void
  on_time_jump(
    const rcl_time_jump_t * time_jump,
    bool before_jump,
    void * user_data);

  /// Private internal storage
  class Impl;

  std::shared_ptr<Impl> impl_;
};

}  // namespace rclcpp

#endif  // RCLCPP__CLOCK_HPP_
