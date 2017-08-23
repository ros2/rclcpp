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

#include "rmw/error_handling.h"
#include "rmw/rmw.h"


using rclcpp::subscription::SubscriptionBase;

SubscriptionBase::SubscriptionBase(
  std::shared_ptr<rcl_node_t> node_handle,
  const rosidl_message_type_support_t & type_support_handle,
  const std::string & topic_name,
  const rcl_subscription_options_t & subscription_options)
: node_handle_(node_handle)
{
  rcl_ret_t ret = rcl_subscription_init(
    &subscription_handle_,
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
  if (rcl_subscription_fini(&subscription_handle_, node_handle_.get()) != RCL_RET_OK) {
    std::stringstream ss;
    ss << "Error in destruction of rcl subscription handle: " <<
      rcl_get_error_string_safe() << '\n';
    (std::cerr << ss.str()).flush();
  }
  if (rcl_subscription_fini(
      &intra_process_subscription_handle_, node_handle_.get()) != RCL_RET_OK)
  {
    std::stringstream ss;
    ss << "Error in destruction of rmw intra process subscription handle: " <<
      rcl_get_error_string_safe() << '\n';
    (std::cerr << ss.str()).flush();
  }
}

const char *
SubscriptionBase::get_topic_name() const
{
  return rcl_subscription_get_topic_name(&subscription_handle_);
}

rcl_subscription_t *
SubscriptionBase::get_subscription_handle()
{
  return &subscription_handle_;
}

const rcl_subscription_t *
SubscriptionBase::get_subscription_handle() const
{
  return &subscription_handle_;
}

const rcl_subscription_t *
SubscriptionBase::get_intra_process_subscription_handle() const
{
  return &intra_process_subscription_handle_;
}
