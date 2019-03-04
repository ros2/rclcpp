// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/subscription.hpp"

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/logging.hpp"

#include "rmw/error_handling.h"
#include "rmw/rmw.h"

using rclcpp::SubscriptionBase;

SubscriptionBase::SubscriptionBase(
  std::shared_ptr<rcl_node_t> node_handle,
  const rosidl_message_type_support_t & type_support_handle,
  const std::string & topic_name,
  const rcl_subscription_options_t & subscription_options,
  bool is_serialized)
: node_handle_(node_handle),
  use_intra_process_(false),
  intra_process_subscription_id_(0),
  type_support_(type_support_handle),
  is_serialized_(is_serialized)
{
  auto custom_deletor = [node_handle](rcl_subscription_t * rcl_subs)
    {
      if (rcl_subscription_fini(rcl_subs, node_handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_node_logger(node_handle.get()).get_child("rclcpp"),
          "Error in destruction of rcl subscription handle: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete rcl_subs;
    };

  auto custom_event_deleter = [](rcl_event_t * event)
    {
      if (rcl_event_fini(event) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "Error in destruction of rcl event handle: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete event;
    };


  subscription_handle_ = std::shared_ptr<rcl_subscription_t>(
    new rcl_subscription_t, custom_deletor);
  *subscription_handle_.get() = rcl_get_zero_initialized_subscription();

  event_handle_ = std::shared_ptr<rcl_event_t>(new rcl_event_t, custom_event_deleter);
  *event_handle_.get() = rcl_get_zero_initialized_event();

  rcl_ret_t ret = rcl_subscription_init(
    subscription_handle_.get(),
    node_handle_.get(),
    &type_support_handle,
    topic_name.c_str(),
    &subscription_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      auto rcl_node_handle = node_handle_.get();
      // this will throw on any validation problem
      rcl_reset_error();
      expand_topic_or_service_name(
        topic_name,
        rcl_node_get_name(rcl_node_handle),
        rcl_node_get_namespace(rcl_node_handle));
    }

    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create subscription");
  }

  ret = rcl_subscription_event_init(get_event_handle().get(), get_subscription_handle().get());
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create subscription event");
  }


  intra_process_subscription_handle_ = std::shared_ptr<rcl_subscription_t>(
    new rcl_subscription_t, custom_deletor);
  *intra_process_subscription_handle_.get() = rcl_get_zero_initialized_subscription();

  intra_process_event_handle_ = std::shared_ptr<rcl_event_t>(new rcl_event_t, custom_event_deleter);
  *intra_process_event_handle_.get() = rcl_get_zero_initialized_event();
}

SubscriptionBase::~SubscriptionBase()
{
  if (!use_intra_process_) {
    return;
  }
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    // TODO(ivanpauno): should this raise an error?
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "Intra process manager died before than a subscription.");
    return;
  }
  ipm->remove_subscription(intra_process_subscription_id_);
}

const char *
SubscriptionBase::get_topic_name() const
{
  return rcl_subscription_get_topic_name(subscription_handle_.get());
}

std::shared_ptr<rcl_subscription_t>
SubscriptionBase::get_subscription_handle()
{
  return subscription_handle_;
}

std::shared_ptr<const rcl_subscription_t>
SubscriptionBase::get_subscription_handle() const
{
  return subscription_handle_;
}

std::shared_ptr<const rcl_subscription_t>
SubscriptionBase::get_intra_process_subscription_handle() const
{
  return intra_process_subscription_handle_;
}

std::shared_ptr<rcl_event_t>
SubscriptionBase::get_event_handle()
{
  return event_handle_;
}

std::shared_ptr<const rcl_event_t>
SubscriptionBase::get_event_handle() const
{
  return event_handle_;
}

size_t
SubscriptionBase::get_number_of_ready_subscriptions()
{
  if (use_intra_process_) {
    return 2;
  } else {
    return 1;
  }
}

size_t
SubscriptionBase::get_number_of_ready_events()
{
  if (use_intra_process_) {
    return 2;
  } else {
    return 1;
  }
}

bool
SubscriptionBase::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  if (rcl_wait_set_add_subscription(wait_set, subscription_handle_.get(),
        &wait_set_subscription_index_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "Couldn't add subscription to wait set: %s", rcl_get_error_string().str);
    return false;
  }

  if (rcl_wait_set_add_event(wait_set, event_handle_.get(), &wait_set_event_index_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "Couldn't add subscription event to wait set: %s", rcl_get_error_string().str);
    return false;
  }

  if (use_intra_process_) {
    if (rcl_wait_set_add_subscription(wait_set, intra_process_subscription_handle_.get(),
          &wait_set_intra_process_subscription_index_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add intra process subscription to wait set: %s", rcl_get_error_string().str);
      return false;
    }

    if (rcl_wait_set_add_event(wait_set, intra_process_event_handle_.get(),
          &wait_set_intra_process_event_index_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "Couldn't add intra process subscription event to wait set: %s", rcl_get_error_string().str);
      return false;
    }
  }

  return true;
}

bool
SubscriptionBase::is_ready(rcl_wait_set_t * wait_set)
{
  subscription_ready_ =
    (wait_set->subscriptions[wait_set_subscription_index_] == subscription_handle_.get());
  intra_process_subscription_ready_ = use_intra_process_ &&
    (wait_set->subscriptions[wait_set_intra_process_subscription_index_] ==
      intra_process_subscription_handle_.get());
  event_ready_ = (wait_set->events[wait_set_event_index_] == event_handle_.get());
  intra_process_event_ready_ =
    (wait_set->events[wait_set_intra_process_event_index_] == intra_process_event_handle_.get());
  return subscription_ready_ || intra_process_subscription_ready_ ||
    event_ready_ || intra_process_event_ready_;
}

void
SubscriptionBase::execute()
{
  if (subscription_ready_) {
    rmw_message_info_t message_info;
    message_info.from_intra_process = false;

    if (is_serialized()) {
      auto serialized_msg = create_serialized_message();
      auto ret = rcl_take_serialized_message(
        get_subscription_handle().get(),
        serialized_msg.get(), &message_info);
      if (RCL_RET_OK == ret) {
        auto void_serialized_msg = std::static_pointer_cast<void>(serialized_msg);
        handle_message(void_serialized_msg, message_info);
      } else if (RCL_RET_SUBSCRIPTION_TAKE_FAILED != ret) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "take_serialized failed for subscription on topic '%s': %s",
          get_topic_name(), rcl_get_error_string().str);
        rcl_reset_error();
      }
      return_serialized_message(serialized_msg);
    } else {
      std::shared_ptr<void> message = create_message();
      auto ret = rcl_take(
        get_subscription_handle().get(),
        message.get(), &message_info);
      if (RCL_RET_OK == ret) {
        handle_message(message, message_info);
      } else if (RCL_RET_SUBSCRIPTION_TAKE_FAILED != ret) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "could not deserialize serialized message on topic '%s': %s",
          get_topic_name(), rcl_get_error_string().str);
        rcl_reset_error();
      }
      return_message(message);
    }
  }

  if (intra_process_subscription_ready_) {
    rcl_interfaces::msg::IntraProcessMessage ipm;
    rmw_message_info_t message_info;
    rcl_ret_t status = rcl_take(
      get_intra_process_subscription_handle().get(),
      &ipm,
      &message_info);

    if (status == RCL_RET_OK) {
      message_info.from_intra_process = true;
      handle_intra_process_message(ipm, message_info);
    } else if (status != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "take failed for intra process subscription on topic '%s': %s",
        get_topic_name(), rcl_get_error_string().str);
      rcl_reset_error();
    }
  }

  if (event_ready_) {
    // rcl_take_event();
    auto example_event = ResourceStatusEvent::LIVELINESS_CHANGED;
    handle_event(example_event);
  }

  if (intra_process_event_ready_) {
    // rcl_take_event();
    auto example_event = ResourceStatusEvent::LIVELINESS_CHANGED;
    handle_event(example_event);
  }
}

const rosidl_message_type_support_t &
SubscriptionBase::get_message_type_support_handle() const
{
  return type_support_;
}

bool
SubscriptionBase::is_serialized() const
{
  return is_serialized_;
}

size_t
SubscriptionBase::get_publisher_count() const
{
  size_t inter_process_publisher_count = 0;

  rmw_ret_t status = rcl_subscription_get_publisher_count(
    subscription_handle_.get(),
    &inter_process_publisher_count);

  if (RCL_RET_OK != status) {
    rclcpp::exceptions::throw_from_rcl_error(status, "failed to get get publisher count");
  }
  return inter_process_publisher_count;
}
