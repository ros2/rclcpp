// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rcl/error_handling.h"
#include "rcl/event.h"

#include "rmw/impl/cpp/demangle.hpp"

#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/event_handler.hpp"
#include "rclcpp/exceptions/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/timer.hpp"

namespace rclcpp
{

UnsupportedEventTypeException::UnsupportedEventTypeException(
  rcl_ret_t ret,
  const rcl_error_state_t * error_state,
  const std::string & prefix)
: UnsupportedEventTypeException(exceptions::RCLErrorBase(ret, error_state), prefix)
{}

UnsupportedEventTypeException::UnsupportedEventTypeException(
  const exceptions::RCLErrorBase & base_exc,
  const std::string & prefix)
: exceptions::RCLErrorBase(base_exc),
  std::runtime_error(prefix + (prefix.empty() ? "" : ": ") + base_exc.formatted_message)
{}

EventHandlerBase::~EventHandlerBase()
{
  if (rcl_event_fini(&event_handle_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "Error in destruction of rcl event handle: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
}

/// Get the number of ready events.
size_t
EventHandlerBase::get_number_of_ready_events()
{
  return 1;
}

/// Add the Waitable to a wait set.
void
EventHandlerBase::add_to_wait_set(rcl_wait_set_t & wait_set)
{
  rcl_ret_t ret = rcl_wait_set_add_event(&wait_set, &event_handle_, &wait_set_event_index_);
  if (RCL_RET_OK != ret) {
    exceptions::throw_from_rcl_error(ret, "Couldn't add event to wait set");
  }
}

/// Check if the Waitable is ready.
bool
EventHandlerBase::is_ready(const rcl_wait_set_t & wait_set)
{
  return wait_set.events[wait_set_event_index_] == &event_handle_;
}

void
EventHandlerBase::set_on_new_event_callback(
  rcl_event_callback_t callback,
  const void * user_data)
{
  rcl_ret_t ret = rcl_event_set_callback(
    &event_handle_,
    callback,
    user_data);

  if (RCL_RET_OK != ret) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(ret, "failed to set the on new message callback for Event");
  }
}

void
EventHandlerBase::set_on_ready_callback(std::function<void(size_t, int)> callback)
{
  if (!callback) {
    throw std::invalid_argument(
            "The callback passed to set_on_ready_callback "
            "is not callable.");
  }

  // Note: we bind the int identifier argument to this waitable's entity types
  auto new_callback =
    [callback, this](size_t number_of_events) {
      try {
        callback(number_of_events, static_cast<int>(EntityType::Event));
      } catch (const std::exception & exception) {
        RCLCPP_ERROR_STREAM(
          // TODO(wjwwood): get this class access to the node logger it is associated with
          rclcpp::get_logger("rclcpp"),
          "rclcpp::EventHandlerBase@" << this <<
            " caught " << rmw::impl::cpp::demangle(exception) <<
            " exception in user-provided callback for the 'on ready' callback: " <<
            exception.what());
      } catch (...) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("rclcpp"),
          "rclcpp::EventHandlerBase@" << this <<
            " caught unhandled exception in user-provided callback " <<
            "for the 'on ready' callback");
      }
    };

  std::lock_guard<std::recursive_mutex> lock(on_new_event_callback_mutex_);

  // Set it temporarily to the new callback, while we replace the old one.
  // This two-step setting, prevents a gap where the old std::function has
  // been replaced but the middleware hasn't been told about the new one yet.
  set_on_new_event_callback(
    rclcpp::detail::cpp_callback_trampoline<decltype(new_callback), const void *, size_t>,
    static_cast<const void *>(&new_callback));

  // Store the std::function to keep it in scope, also overwrites the existing one.
  on_new_event_callback_ = new_callback;

  // Set it again, now using the permanent storage.
  set_on_new_event_callback(
    rclcpp::detail::cpp_callback_trampoline<
      decltype(on_new_event_callback_), const void *, size_t>,
    static_cast<const void *>(&on_new_event_callback_));
}

void
EventHandlerBase::clear_on_ready_callback()
{
  std::lock_guard<std::recursive_mutex> lock(on_new_event_callback_mutex_);
  if (on_new_event_callback_) {
    set_on_new_event_callback(nullptr, nullptr);
    on_new_event_callback_ = nullptr;
  }
}

std::vector<std::shared_ptr<rclcpp::TimerBase>>
EventHandlerBase::get_timers() const
{
  return {};
}

}  // namespace rclcpp
