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
: type_(group_type), can_be_taken_from_(true)
{}

const std::vector<rclcpp::SubscriptionBase::WeakPtr> &
CallbackGroup::get_subscription_ptrs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return subscription_ptrs_;
}

const std::vector<rclcpp::TimerBase::WeakPtr> &
CallbackGroup::get_timer_ptrs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return timer_ptrs_;
}

const std::vector<rclcpp::ServiceBase::WeakPtr> &
CallbackGroup::get_service_ptrs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return service_ptrs_;
}

const std::vector<rclcpp::ClientBase::WeakPtr> &
CallbackGroup::get_client_ptrs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return client_ptrs_;
}

const std::vector<rclcpp::Waitable::WeakPtr> &
CallbackGroup::get_waitable_ptrs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return waitable_ptrs_;
}

std::atomic_bool &
CallbackGroup::can_be_taken_from()
{
  return can_be_taken_from_;
}

const CallbackGroupType &
CallbackGroup::type() const
{
  return type_;
}

void
CallbackGroup::add_subscription(
  const rclcpp::SubscriptionBase::SharedPtr subscription_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  subscription_ptrs_.push_back(subscription_ptr);
}

void
CallbackGroup::add_timer(const rclcpp::TimerBase::SharedPtr timer_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timer_ptrs_.push_back(timer_ptr);
}

void
CallbackGroup::add_service(const rclcpp::ServiceBase::SharedPtr service_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  service_ptrs_.push_back(service_ptr);
}

void
CallbackGroup::add_client(const rclcpp::ClientBase::SharedPtr client_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  client_ptrs_.push_back(client_ptr);
}

void
CallbackGroup::add_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  waitable_ptrs_.push_back(waitable_ptr);
}

void
CallbackGroup::remove_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto iter = waitable_ptrs_.begin(); iter != waitable_ptrs_.end(); ++iter) {
    const auto shared_ptr = iter->lock();
    if (shared_ptr.get() == waitable_ptr.get()) {
      waitable_ptrs_.erase(iter);
      break;
    }
  }
}
