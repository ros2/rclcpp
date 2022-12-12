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

#ifndef RCLCPP__QOS_EVENT_HPP_
#define RCLCPP__QOS_EVENT_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/event_callback.h"
#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/incompatible_qos_events_statuses.h"

#include "rcutils/logging_macros.h"

#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/function_traits.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{

using QOSDeadlineRequestedInfo = rmw_requested_deadline_missed_status_t;
using QOSDeadlineOfferedInfo = rmw_offered_deadline_missed_status_t;
using QOSLivelinessChangedInfo = rmw_liveliness_changed_status_t;
using QOSLivelinessLostInfo = rmw_liveliness_lost_status_t;
using QOSMessageLostInfo = rmw_message_lost_status_t;
using QOSOfferedIncompatibleQoSInfo = rmw_offered_qos_incompatible_event_status_t;
using QOSRequestedIncompatibleQoSInfo = rmw_requested_qos_incompatible_event_status_t;

using QOSDeadlineRequestedCallbackType = std::function<void (QOSDeadlineRequestedInfo &)>;
using QOSDeadlineOfferedCallbackType = std::function<void (QOSDeadlineOfferedInfo &)>;
using QOSLivelinessChangedCallbackType = std::function<void (QOSLivelinessChangedInfo &)>;
using QOSLivelinessLostCallbackType = std::function<void (QOSLivelinessLostInfo &)>;
using QOSMessageLostCallbackType = std::function<void (QOSMessageLostInfo &)>;
using QOSOfferedIncompatibleQoSCallbackType = std::function<void (QOSOfferedIncompatibleQoSInfo &)>;
using QOSRequestedIncompatibleQoSCallbackType =
  std::function<void (QOSRequestedIncompatibleQoSInfo &)>;

/// Contains callbacks for various types of events a Publisher can receive from the middleware.
struct PublisherEventCallbacks
{
  QOSDeadlineOfferedCallbackType deadline_callback;
  QOSLivelinessLostCallbackType liveliness_callback;
  QOSOfferedIncompatibleQoSCallbackType incompatible_qos_callback;
};

/// Contains callbacks for non-message events that a Subscription can receive from the middleware.
struct SubscriptionEventCallbacks
{
  QOSDeadlineRequestedCallbackType deadline_callback;
  QOSLivelinessChangedCallbackType liveliness_callback;
  QOSRequestedIncompatibleQoSCallbackType incompatible_qos_callback;
  QOSMessageLostCallbackType message_lost_callback;
};

class UnsupportedEventTypeException : public exceptions::RCLErrorBase, public std::runtime_error
{
public:
  RCLCPP_PUBLIC
  UnsupportedEventTypeException(
    rcl_ret_t ret,
    const rcl_error_state_t * error_state,
    const std::string & prefix);

  RCLCPP_PUBLIC
  UnsupportedEventTypeException(
    const exceptions::RCLErrorBase & base_exc,
    const std::string & prefix);
};

class QOSEventHandlerBase : public Waitable
{
public:
  enum class EntityType : std::size_t
  {
    Event,
  };

  RCLCPP_PUBLIC
  virtual ~QOSEventHandlerBase();

  /// Get the number of ready events
  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_events() override;

  /// Add the Waitable to a wait set.
  RCLCPP_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  /// Check if the Waitable is ready.
  RCLCPP_PUBLIC
  bool
  is_ready(rcl_wait_set_t * wait_set) override;

  /// Set a callback to be called when each new event instance occurs.
  /**
   * The callback receives a size_t which is the number of events that occurred
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if events occurred before any
   * callback was set.
   *
   * The callback also receives an int identifier argument.
   * This is needed because a Waitable may be composed of several distinct entities,
   * such as subscriptions, services, etc.
   * The application should provide a generic callback function that will be then
   * forwarded by the waitable to all of its entities.
   * Before forwarding, a different value for the identifier argument will be
   * bond to the function.
   * This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.
   *
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * Calling it again will clear any previously set callback.
   *
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the qos event
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rmw_event_set_callback
   * \sa rcl_event_set_callback
   *
   * \param[in] callback functor to be called when a new event occurs
   */
  void
  set_on_ready_callback(std::function<void(size_t, int)> callback) override
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
            "rclcpp::QOSEventHandlerBase@" << this <<
              " caught " << rmw::impl::cpp::demangle(exception) <<
              " exception in user-provided callback for the 'on ready' callback: " <<
              exception.what());
        } catch (...) {
          RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("rclcpp"),
            "rclcpp::QOSEventHandlerBase@" << this <<
              " caught unhandled exception in user-provided callback " <<
              "for the 'on ready' callback");
        }
      };

    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // Set it temporarily to the new callback, while we replace the old one.
    // This two-step setting, prevents a gap where the old std::function has
    // been replaced but the middleware hasn't been told about the new one yet.
    set_on_new_event_callback(
      rclcpp::detail::cpp_callback_trampoline<const void *, size_t>,
      static_cast<const void *>(&new_callback));

    // Store the std::function to keep it in scope, also overwrites the existing one.
    on_new_event_callback_ = new_callback;

    // Set it again, now using the permanent storage.
    set_on_new_event_callback(
      rclcpp::detail::cpp_callback_trampoline<const void *, size_t>,
      static_cast<const void *>(&on_new_event_callback_));
  }

  /// Unset the callback registered for new events, if any.
  void
  clear_on_ready_callback() override
  {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);
    if (on_new_event_callback_) {
      set_on_new_event_callback(nullptr, nullptr);
      on_new_event_callback_ = nullptr;
    }
  }

protected:
  RCLCPP_PUBLIC
  void
  set_on_new_event_callback(rcl_event_callback_t callback, const void * user_data);

  rcl_event_t event_handle_;
  size_t wait_set_event_index_;
  std::recursive_mutex callback_mutex_;
  std::function<void(size_t)> on_new_event_callback_{nullptr};
};

template<typename EventCallbackT, typename ParentHandleT>
class QOSEventHandler : public QOSEventHandlerBase
{
public:
  template<typename InitFuncT, typename EventTypeEnum>
  QOSEventHandler(
    const EventCallbackT & callback,
    InitFuncT init_func,
    ParentHandleT parent_handle,
    EventTypeEnum event_type)
  : parent_handle_(parent_handle), event_callback_(callback)
  {
    event_handle_ = rcl_get_zero_initialized_event();
    rcl_ret_t ret = init_func(&event_handle_, parent_handle.get(), event_type);
    if (ret != RCL_RET_OK) {
      if (ret == RCL_RET_UNSUPPORTED) {
        UnsupportedEventTypeException exc(ret, rcl_get_error_state(), "Failed to initialize event");
        rcl_reset_error();
        throw exc;
      } else {
        rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to initialize event");
      }
    }
  }

  /// Take data so that the callback cannot be scheduled again
  std::shared_ptr<void>
  take_data() override
  {
    EventCallbackInfoT callback_info;
    rcl_ret_t ret = rcl_take_event(&event_handle_, &callback_info);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't take event info: %s", rcl_get_error_string().str);
      return nullptr;
    }
    return std::static_pointer_cast<void>(std::make_shared<EventCallbackInfoT>(callback_info));
  }

  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override
  {
    (void)id;
    return take_data();
  }

  /// Execute any entities of the Waitable that are ready.
  void
  execute(std::shared_ptr<void> & data) override
  {
    if (!data) {
      throw std::runtime_error("'data' is empty");
    }
    auto callback_ptr = std::static_pointer_cast<EventCallbackInfoT>(data);
    event_callback_(*callback_ptr);
    callback_ptr.reset();
  }

private:
  using EventCallbackInfoT = typename std::remove_reference<typename
      rclcpp::function_traits::function_traits<EventCallbackT>::template argument_type<0>>::type;

  ParentHandleT parent_handle_;
  EventCallbackT event_callback_;
};

}  // namespace rclcpp

#endif  // RCLCPP__QOS_EVENT_HPP_
