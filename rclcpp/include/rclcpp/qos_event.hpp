// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__QOS_EVENT_HPP_
#define RCLCPP__QOS_EVENT_HPP_

#include <functional>

#include "rcl/error_handling.h"

#include "rcutils/logging_macros.h"

#include "rclcpp/waitable.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/function_traits.hpp"


namespace rclcpp
{

struct QOSDeadlineEventInfo
{
  enum DeadlineEventType
  {
    DEADLINE_MISSED
  };

  DeadlineEventType event_type;
  void * other_info;
};

struct QOSLivelinessEventInfo
{
  enum LivelinessEventType
  {
    LIVELINESS_CHANGED
  };

  LivelinessEventType event_type;
  rmw_liveliness_lost_t * other_info;
};

struct QOSLifespanEventInfo
{
  enum LifespanEventType
  {
    LIFESPAN_EXPIRED
  };

  LifespanEventType event_type;
  void * other_info;
};


using QOSDeadlineEventCallbackType = std::function<void (QOSDeadlineEventInfo &)>;
using QOSLivelinessEventCallbackType = std::function<void (QOSLivelinessEventInfo &)>;
using QOSLifespanEventCallbackType = std::function<void (QOSLifespanEventInfo &)>;


// struct QOSDeadlineEventConfig {
//   using CallbackT = QOSDeadlineEventCallbackType;
//   using EventInfoT = QOSDeadlineEventInfo;
// };

// struct QOSLivelinessEventConfig {
//   using CallbackT = QOSLivelinessEventCallbackType;
//   using EventInfoT = QOSLivelinessEventInfo;
// };

// struct QOSLifespanEventConfig {
//   using CallbackT = QOSLifespanEventCallbackType;
//   using EventInfoT = QOSLifespanEventInfo;
// };


class QOSEventBase : public Waitable
{
public:
  /// Get the number of ready events
  size_t
  get_number_of_ready_events() override;

  /// Add the Waitable to a wait set.
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  /// Check if the Waitable is ready.
  bool
  is_ready(rcl_wait_set_t * wait_set) override;

protected:
  rcl_event_t event_handle_;
  size_t wait_set_event_index_;
};


template<typename EventCallbackT>
class QOSEvent : public QOSEventBase
{
public:
  template<typename InitFuncT, typename HandleT, typename OptionsT, typename EventTypeEnum>
  QOSEvent(
    const EventCallbackT & callback,
    InitFuncT init_func,
    HandleT handle,
    OptionsT options,
    EventTypeEnum type)
  : event_callback_(callback)
  {
    event_handle_ = rcl_get_zero_initialized_event();
    rcl_ret_t ret = init_func(&event_handle_, handle, options, type);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "could not create event");
    }
  }

  ~QOSEvent()
  {
    if (rcl_event_fini(&event_handle_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Error in destruction of rcl event handle: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }

  /// Execute any entities of the Waitable that are ready.
  void
  execute() override
  {
    EventCallbackInfoT callback_info;

    rcl_ret_t ret = RCL_RET_ERROR;
    // rcl_ret_t ret = rcl_take_event(&event_handle_, callback_info.other_info);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't take event info: %s", rcl_get_error_string().str);
      return;
    }

    event_callback_(callback_info);
  }

private:
  using EventCallbackInfoT = typename std::remove_reference<typename
      rclcpp::function_traits::function_traits<EventCallbackT>::template argument_type<0>>::type;

  EventCallbackT event_callback_;
};

}  // namespace rclcpp

#endif  // RCLCPP__QOS_EVENT_HPP_
