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

#include "rclcpp/clock.hpp"

#include <condition_variable>
#include <memory>
#include <thread>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/utilities.hpp"

#include "rcpputils/scope_exit.hpp"
#include "rcutils/logging_macros.h"

namespace rclcpp
{

class Clock::Impl
{
public:
  explicit Impl(rcl_clock_type_t clock_type)
  : allocator_{rcl_get_default_allocator()}
  {
    rcl_ret_t ret = rcl_clock_init(clock_type, &rcl_clock_, &allocator_);
    if (ret != RCL_RET_OK) {
      exceptions::throw_from_rcl_error(ret, "failed to initialize rcl clock");
    }
  }

  ~Impl()
  {
    rcl_ret_t ret = rcl_clock_fini(&rcl_clock_);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to fini rcl clock.");
    }
  }

  rcl_clock_t rcl_clock_;
  rcl_allocator_t allocator_;
  rcpputils::PIMutex clock_mutex_;
};

JumpHandler::JumpHandler(
  pre_callback_t pre_callback,
  post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
: pre_callback(pre_callback),
  post_callback(post_callback),
  notice_threshold(threshold)
{}

Clock::Clock(rcl_clock_type_t clock_type)
: impl_(new Clock::Impl(clock_type)) {}

Clock::~Clock() {}

Time
Clock::now() const
{
  Time now(0, 0, impl_->rcl_clock_.type);

  auto ret = rcl_clock_get_now(&impl_->rcl_clock_, &now.rcl_time_.nanoseconds);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(ret, "could not get current time stamp");
  }

  return now;
}

bool
Clock::sleep_until(Time until, Context::SharedPtr context)
{
  if (!context || !context->is_valid()) {
    throw std::runtime_error("context cannot be slept with because it's invalid");
  }
  const auto this_clock_type = get_clock_type();
  if (until.get_clock_type() != this_clock_type) {
    throw std::runtime_error("until's clock type does not match this clock's type");
  }
  bool time_source_changed = false;

  std::condition_variable cv;

  // Wake this thread if the context is shutdown
  rclcpp::OnShutdownCallbackHandle shutdown_cb_handle = context->add_on_shutdown_callback(
    [&cv]() {
      cv.notify_one();
    });
  // No longer need the shutdown callback when this function exits
  auto callback_remover = rcpputils::scope_exit(
    [context, &shutdown_cb_handle]() {
      context->remove_on_shutdown_callback(shutdown_cb_handle);
    });

  if (this_clock_type == RCL_STEADY_TIME) {
    // Synchronize because RCL steady clock epoch might differ from chrono::steady_clock epoch
    const Time rcl_entry = now();
    const std::chrono::steady_clock::time_point chrono_entry = std::chrono::steady_clock::now();
    const Duration delta_t = until - rcl_entry;
    const std::chrono::steady_clock::time_point chrono_until =
      chrono_entry + std::chrono::nanoseconds(delta_t.nanoseconds());

    // loop over spurious wakeups but notice shutdown
    std::unique_lock<std::mutex> lock(impl_->clock_mutex_);
    while (now() < until && context->is_valid()) {
      cv.wait_until(lock, chrono_until);
    }
  } else if (this_clock_type == RCL_SYSTEM_TIME) {
    auto system_time = std::chrono::system_clock::time_point(
      // Cast because system clock resolution is too big for nanoseconds on some systems
      std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::nanoseconds(until.nanoseconds())));

    // loop over spurious wakeups but notice shutdown
    std::unique_lock<std::mutex> lock(impl_->clock_mutex_);
    while (now() < until && context->is_valid()) {
      cv.wait_until(lock, system_time);
    }
  } else if (this_clock_type == RCL_ROS_TIME) {
    // Install jump handler for any amount of time change, for two purposes:
    // - if ROS time is active, check if time reached on each new clock sample
    // - Trigger via on_clock_change to detect if time source changes, to invalidate sleep
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = true;
    // 0 is disable, so -1 and 1 are smallest possible time changes
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 1;
    auto clock_handler = create_jump_callback(
      nullptr,
      [&cv, &time_source_changed](const rcl_time_jump_t & jump) {
        if (jump.clock_change != RCL_ROS_TIME_NO_CHANGE) {
          time_source_changed = true;
        }
        cv.notify_one();
      },
      threshold);

    if (!ros_time_is_active()) {
      auto system_time = std::chrono::system_clock::time_point(
        // Cast because system clock resolution is too big for nanoseconds on some systems
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
          std::chrono::nanoseconds(until.nanoseconds())));

      // loop over spurious wakeups but notice shutdown or time source change
      std::unique_lock<std::mutex> lock(impl_->clock_mutex_);
      while (now() < until && context->is_valid() && !time_source_changed) {
        cv.wait_until(lock, system_time);
      }
    } else {
      // RCL_ROS_TIME with ros_time_is_active.
      // Just wait without "until" because installed
      // jump callbacks wake the cv on every new sample.
      std::unique_lock<std::mutex> lock(impl_->clock_mutex_);
      while (now() < until && context->is_valid() && !time_source_changed) {
        cv.wait(lock);
      }
    }
  }

  if (!context->is_valid() || time_source_changed) {
    return false;
  }

  return now() >= until;
}

bool
Clock::sleep_for(Duration rel_time, Context::SharedPtr context)
{
  return sleep_until(now() + rel_time, context);
}

bool
Clock::started()
{
  if (!rcl_clock_valid(get_clock_handle())) {
    throw std::runtime_error("clock is not rcl_clock_valid");
  }
  return rcl_clock_time_started(get_clock_handle());
}

bool
Clock::wait_until_started(Context::SharedPtr context)
{
  if (!context || !context->is_valid()) {
    throw std::runtime_error("context cannot be slept with because it's invalid");
  }
  if (!rcl_clock_valid(get_clock_handle())) {
    throw std::runtime_error("clock cannot be waited on as it is not rcl_clock_valid");
  }

  if (started()) {
    return true;
  } else {
    // Wait until the first non-zero time
    return sleep_until(rclcpp::Time(0, 1, get_clock_type()), context);
  }
}

bool
Clock::wait_until_started(
  const Duration & timeout,
  Context::SharedPtr context,
  const Duration & wait_tick_ns)
{
  if (!context || !context->is_valid()) {
    throw std::runtime_error("context cannot be slept with because it's invalid");
  }
  if (!rcl_clock_valid(get_clock_handle())) {
    throw std::runtime_error("clock cannot be waited on as it is not rcl_clock_valid");
  }

  Clock timeout_clock = Clock(RCL_STEADY_TIME);
  Time start = timeout_clock.now();

  // Check if the clock has started every wait_tick_ns nanoseconds
  // Context check checks for rclcpp::shutdown()
  while (!started() && context->is_valid()) {
    if (timeout < wait_tick_ns) {
      timeout_clock.sleep_for(timeout);
    } else {
      Duration time_left = start + timeout - timeout_clock.now();
      if (time_left > wait_tick_ns) {
        timeout_clock.sleep_for(Duration(wait_tick_ns));
      } else {
        timeout_clock.sleep_for(time_left);
      }
    }

    if (timeout_clock.now() - start > timeout) {
      return started();
    }
  }
  return started();
}


bool
Clock::ros_time_is_active()
{
  if (!rcl_clock_valid(&impl_->rcl_clock_)) {
    RCUTILS_LOG_ERROR("ROS time not valid!");
    return false;
  }

  bool is_enabled = false;
  auto ret = rcl_is_enabled_ros_time_override(&impl_->rcl_clock_, &is_enabled);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(
      ret, "Failed to check ros_time_override_status");
  }
  return is_enabled;
}

rcl_clock_t *
Clock::get_clock_handle() noexcept
{
  return &impl_->rcl_clock_;
}

rcl_clock_type_t
Clock::get_clock_type() const noexcept
{
  return impl_->rcl_clock_.type;
}

rcpputils::PIMutex &
Clock::get_clock_mutex() noexcept
{
  return impl_->clock_mutex_;
}

void
Clock::on_time_jump(
  const rcl_time_jump_t * time_jump,
  bool before_jump,
  void * user_data)
{
  const auto * handler = static_cast<JumpHandler *>(user_data);
  if (nullptr == handler) {
    return;
  }
  if (before_jump && handler->pre_callback) {
    handler->pre_callback();
  } else if (!before_jump && handler->post_callback) {
    handler->post_callback(*time_jump);
  }
}

JumpHandler::SharedPtr
Clock::create_jump_callback(
  JumpHandler::pre_callback_t pre_callback,
  JumpHandler::post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
{
  // Allocate a new jump handler
  JumpHandler::UniquePtr handler(new JumpHandler(pre_callback, post_callback, threshold));
  if (nullptr == handler) {
    throw std::bad_alloc{};
  }

  {
    std::lock_guard<std::mutex> clock_guard(impl_->clock_mutex_);
    // Try to add the jump callback to the clock
    rcl_ret_t ret = rcl_clock_add_jump_callback(
      &impl_->rcl_clock_, threshold, Clock::on_time_jump,
      handler.get());
    if (RCL_RET_OK != ret) {
      exceptions::throw_from_rcl_error(ret, "Failed to add time jump callback");
    }
  }

  std::weak_ptr<Clock::Impl> weak_impl = impl_;
  // *INDENT-OFF*
  // create shared_ptr that removes the callback automatically when all copies are destructed
  return JumpHandler::SharedPtr(handler.release(), [weak_impl](JumpHandler * handler) noexcept {
    auto shared_impl = weak_impl.lock();
    if (shared_impl) {
      std::lock_guard<std::mutex> clock_guard(shared_impl->clock_mutex_);
      rcl_ret_t ret = rcl_clock_remove_jump_callback(&shared_impl->rcl_clock_,
          Clock::on_time_jump, handler);
      if (RCL_RET_OK != ret) {
        RCUTILS_LOG_ERROR("Failed to remove time jump callback");
      }
    }
    delete handler;
  });
  // *INDENT-ON*
}

}  // namespace rclcpp
