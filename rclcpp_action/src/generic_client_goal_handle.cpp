// Copyright 2025 Sony Group Corporation.
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

#include <cstdint>
#include <future>
#include <mutex>

#include "rclcpp_action/generic_client_goal_handle.hpp"

#include "rclcpp/logging.hpp"

namespace rclcpp_action
{
GenericClientGoalHandle::GenericClientGoalHandle(
  const GoalInfo & info, FeedbackCallback feedback_callback, ResultCallback result_callback)
: info_(info),
  result_future_(result_promise_.get_future()),
  feedback_callback_(feedback_callback),
  result_callback_(result_callback)
{
}

GenericClientGoalHandle::~GenericClientGoalHandle()
{}

const GoalUUID &
GenericClientGoalHandle::get_goal_id() const
{
  return info_.goal_id.uuid;
}

rclcpp::Time
GenericClientGoalHandle::get_goal_stamp() const
{
  return info_.stamp;
}

std::shared_future<typename GenericClientGoalHandle::WrappedResult>
GenericClientGoalHandle::async_get_result()
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  if (!is_result_aware_) {
    throw exceptions::UnawareGoalHandleError();
  }
  return result_future_;
}

void
GenericClientGoalHandle::set_result(const WrappedResult & wrapped_result)
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  status_ = static_cast<int8_t>(wrapped_result.code);
  result_promise_.set_value(wrapped_result);
  if (result_callback_) {
    result_callback_(wrapped_result);
    result_callback_ = ResultCallback();
  }
}

void
GenericClientGoalHandle::set_feedback_callback(FeedbackCallback callback)
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  feedback_callback_ = callback;
}

void
GenericClientGoalHandle::set_result_callback(ResultCallback callback)
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  result_callback_ = callback;
}

int8_t
GenericClientGoalHandle::get_status()
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  return status_;
}

void
GenericClientGoalHandle::set_status(int8_t status)
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  status_ = status;
}

bool
GenericClientGoalHandle::is_feedback_aware()
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  return feedback_callback_ != nullptr;
}

bool
GenericClientGoalHandle::is_result_aware()
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  return is_result_aware_;
}

bool
GenericClientGoalHandle::set_result_awareness(bool awareness)
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  bool previous = is_result_aware_;
  is_result_aware_ = awareness;
  return previous;
}

void
GenericClientGoalHandle::invalidate(const exceptions::UnawareGoalHandleError & ex)
{
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  // Guard against multiple calls
  if (is_invalidated()) {
    return;
  }
  is_result_aware_ = false;
  invalidate_exception_ = std::make_exception_ptr(ex);
  status_ = GoalStatus::STATUS_UNKNOWN;
  result_promise_.set_exception(invalidate_exception_);
}

bool
GenericClientGoalHandle::is_invalidated() const
{
  return invalidate_exception_ != nullptr;
}

void
GenericClientGoalHandle::call_feedback_callback(
  typename GenericClientGoalHandle::SharedPtr shared_this,
  const void * feedback_message)
{
  if (shared_this.get() != this) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp_action"), "Sent feedback to wrong goal handle.");
    return;
  }
  std::lock_guard<std::recursive_mutex> guard(handle_mutex_);
  if (nullptr == feedback_callback_) {
    // Normal, some feedback messages may arrive after the goal result.
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_action"), "Received feedback but goal ignores it.");
    return;
  }
  feedback_callback_(shared_this, feedback_message);
}
}  // namespace rclcpp_action
