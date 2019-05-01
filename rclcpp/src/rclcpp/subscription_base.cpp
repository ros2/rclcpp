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

#include "rclcpp/subscription_base.hpp"

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

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

  subscription_handle_ = std::shared_ptr<rcl_subscription_t>(
    new rcl_subscription_t, custom_deletor);
  *subscription_handle_.get() = rcl_get_zero_initialized_subscription();

  intra_process_subscription_handle_ = std::shared_ptr<rcl_subscription_t>(
    new rcl_subscription_t, custom_deletor);
  *intra_process_subscription_handle_.get() = rcl_get_zero_initialized_subscription();

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

const std::shared_ptr<rcl_subscription_t>
SubscriptionBase::get_subscription_handle() const
{
  return subscription_handle_;
}

const std::shared_ptr<rcl_subscription_t>
SubscriptionBase::get_intra_process_subscription_handle() const
{
  return intra_process_subscription_handle_;
}

const std::vector<std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
SubscriptionBase::get_event_handlers() const
{
  return event_handlers_;
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

void SubscriptionBase::setup_intra_process(
  uint64_t intra_process_subscription_id,
  IntraProcessManagerWeakPtr weak_ipm,
  const rcl_subscription_options_t & intra_process_options)
{
  std::string intra_process_topic_name = std::string(get_topic_name()) + "/_intra";
  rcl_ret_t ret = rcl_subscription_init(
    intra_process_subscription_handle_.get(),
    node_handle_.get(),
    rclcpp::type_support::get_intra_process_message_msg_type_support(),
    intra_process_topic_name.c_str(),
    &intra_process_options);
  if (ret != RCL_RET_OK) {
    if (ret == RCL_RET_TOPIC_NAME_INVALID) {
      auto rcl_node_handle = node_handle_.get();
      // this will throw on any validation problem
      rcl_reset_error();
      expand_topic_or_service_name(
        intra_process_topic_name,
        rcl_node_get_name(rcl_node_handle),
        rcl_node_get_namespace(rcl_node_handle));
    }

    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create intra process subscription");
  }

  intra_process_subscription_id_ = intra_process_subscription_id;
  weak_ipm_ = weak_ipm;
  use_intra_process_ = true;
}
