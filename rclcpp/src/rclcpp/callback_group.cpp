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

#include "rclcpp/callback_group.hpp"

#include <vector>

using rclcpp::callback_group::CallbackGroup;
using rclcpp::callback_group::CallbackGroupType;

CallbackGroup::CallbackGroup(CallbackGroupType group_type)
: type_(group_type), can_be_taken_from(true)
{}

const std::vector<rclcpp::subscription::SubscriptionBase::WeakPtr> &
CallbackGroup::get_subscription_ptrs() const
{
  return subscription_ptrs_;
}

const std::vector<rclcpp::timer::TimerBase::WeakPtr> &
CallbackGroup::get_timer_ptrs() const
{
  return timer_ptrs_;
}

const std::vector<rclcpp::service::ServiceBase::SharedPtr> &
CallbackGroup::get_service_ptrs() const
{
  return service_ptrs_;
}

const std::vector<rclcpp::client::ClientBase::SharedPtr> &
CallbackGroup::get_client_ptrs() const
{
  return client_ptrs_;
}

/*
std::atomic_bool &
CallbackGroup::can_be_taken_from()
{
  return can_be_taken_from_;
}
*/

const CallbackGroupType &
CallbackGroup::type() const
{
  return type_;
}

void
CallbackGroup::add_subscription(
  const rclcpp::subscription::SubscriptionBase::SharedPtr subscription_ptr)
{
  subscription_ptrs_.push_back(subscription_ptr);
}

void
CallbackGroup::add_timer(const rclcpp::timer::TimerBase::SharedPtr timer_ptr)
{
  timer_ptrs_.push_back(timer_ptr);
}

void
CallbackGroup::add_service(const rclcpp::service::ServiceBase::SharedPtr service_ptr)
{
  service_ptrs_.push_back(service_ptr);
}

void
CallbackGroup::add_client(const rclcpp::client::ClientBase::SharedPtr client_ptr)
{
  client_ptrs_.push_back(client_ptr);
}
