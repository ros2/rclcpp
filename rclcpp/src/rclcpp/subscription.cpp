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
#include <string>

#include "rmw/error_handling.h"
#include "rmw/rmw.h"


using rclcpp::subscription::SubscriptionBase;

SubscriptionBase::SubscriptionBase(
  std::shared_ptr<rmw_node_t> node_handle,
  rmw_subscription_t * subscription_handle,
  const std::string & topic_name,
  bool ignore_local_publications)
: intra_process_subscription_handle_(nullptr),
  node_handle_(node_handle),
  subscription_handle_(subscription_handle),
  topic_name_(topic_name),
  ignore_local_publications_(ignore_local_publications)
{
  // To avoid unused private member warnings.
  (void)ignore_local_publications_;
}

SubscriptionBase::~SubscriptionBase()
{
  if (subscription_handle_) {
    if (rmw_destroy_subscription(node_handle_.get(), subscription_handle_) != RMW_RET_OK) {
      std::stringstream ss;
      ss << "Error in destruction of rmw subscription handle: " <<
        rmw_get_error_string_safe() << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
  if (intra_process_subscription_handle_) {
    auto ret = rmw_destroy_subscription(node_handle_.get(), intra_process_subscription_handle_);
    if (ret != RMW_RET_OK) {
      std::stringstream ss;
      ss << "Error in destruction of rmw intra process subscription handle: " <<
        rmw_get_error_string_safe() << '\n';
      (std::cerr << ss.str()).flush();
    }
  }
}

const std::string &
SubscriptionBase::get_topic_name() const
{
  return this->topic_name_;
}

const rmw_subscription_t *
SubscriptionBase::get_subscription_handle() const
{
  return subscription_handle_;
}

const rmw_subscription_t *
SubscriptionBase::get_intra_process_subscription_handle() const
{
  return intra_process_subscription_handle_;
}
