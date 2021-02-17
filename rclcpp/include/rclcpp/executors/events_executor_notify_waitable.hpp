// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__EVENTS_EXECUTOR_NOTIFY_WAITABLE_HPP_
#define RCLCPP__EXECUTORS__EVENTS_EXECUTOR_NOTIFY_WAITABLE_HPP_

#include <list>
#include <memory>

#include "rcl/guard_condition.h"
#include "rclcpp/executors/event_waitable.hpp"

namespace rclcpp
{
namespace executors
{

/**
 * @brief This class provides an EventWaitable that allows to
 * wake up an EventsExecutor when a guard condition is notified.
 */
class EventsExecutorNotifyWaitable final : public EventWaitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EventsExecutorNotifyWaitable)

  // Constructor
  RCLCPP_PUBLIC
  EventsExecutorNotifyWaitable() = default;

  // Destructor
  RCLCPP_PUBLIC
  virtual ~EventsExecutorNotifyWaitable() = default;

  // The function is a no-op, since we only care of waking up the executor
  RCLCPP_PUBLIC
  void
  execute(std::shared_ptr<void> & data) override
  {
    (void)data;
  }

  RCLCPP_PUBLIC
  void
  add_guard_condition(const rcl_guard_condition_t * guard_condition)
  {
    notify_guard_conditions_.push_back(guard_condition);
  }

  RCLCPP_PUBLIC
  void
  set_events_executor_callback(
    rclcpp::executors::EventsExecutor * executor,
    rmw_listener_callback_t executor_callback) const override
  {
    for (auto gc : notify_guard_conditions_) {
      rcl_ret_t ret = rcl_guard_condition_set_listener_callback(
        gc,
        executor_callback,
        executor,
        this,
        false);

      if (RCL_RET_OK != ret) {
        throw std::runtime_error("Couldn't set guard condition events callback");
      }
    }
  }

  RCLCPP_PUBLIC
  std::shared_ptr<void>
  take_data()
  {
    // This waitable doesn't handle any data
    return nullptr;
  }

private:
  std::list<const rcl_guard_condition_t *> notify_guard_conditions_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENTS_EXECUTOR_NOTIFY_WAITABLE_HPP_