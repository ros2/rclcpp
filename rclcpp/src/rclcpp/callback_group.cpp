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

#include <algorithm>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/waitable.hpp"

using rclcpp::CallbackGroup;
using rclcpp::CallbackGroupType;

CallbackGroup::CallbackGroup(
  CallbackGroupType group_type,
  bool automatically_add_to_executor_with_node)
: type_(group_type), associated_with_executor_(false),
  can_be_taken_from_(true),
  automatically_add_to_executor_with_node_(automatically_add_to_executor_with_node)
{}

CallbackGroup::~CallbackGroup()
{
  trigger_notify_guard_condition();
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

void CallbackGroup::collect_all_ptrs(
  std::function<void(const rclcpp::SubscriptionBase::SharedPtr &)> sub_func,
  std::function<void(const rclcpp::ServiceBase::SharedPtr &)> service_func,
  std::function<void(const rclcpp::ClientBase::SharedPtr &)> client_func,
  std::function<void(const rclcpp::TimerBase::SharedPtr &)> timer_func,
  std::function<void(const rclcpp::Waitable::SharedPtr &)> waitable_func) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const rclcpp::SubscriptionBase::WeakPtr & weak_ptr : subscription_ptrs_) {
    rclcpp::SubscriptionBase::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      sub_func(ref_ptr);
    }
  }

  for (const rclcpp::ServiceBase::WeakPtr & weak_ptr : service_ptrs_) {
    rclcpp::ServiceBase::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      service_func(ref_ptr);
    }
  }

  for (const rclcpp::ClientBase::WeakPtr & weak_ptr : client_ptrs_) {
    rclcpp::ClientBase::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      client_func(ref_ptr);
    }
  }

  for (const rclcpp::TimerBase::WeakPtr & weak_ptr : timer_ptrs_) {
    rclcpp::TimerBase::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      timer_func(ref_ptr);
    }
  }

  for (const rclcpp::Waitable::WeakPtr & weak_ptr : waitable_ptrs_) {
    rclcpp::Waitable::SharedPtr ref_ptr = weak_ptr.lock();
    if (ref_ptr) {
      waitable_func(ref_ptr);
    }
  }
}

std::atomic_bool &
CallbackGroup::get_associated_with_executor_atomic()
{
  return associated_with_executor_;
}

bool
CallbackGroup::automatically_add_to_executor_with_node() const
{
  return automatically_add_to_executor_with_node_;
}

rclcpp::GuardCondition::SharedPtr
CallbackGroup::get_notify_guard_condition(const rclcpp::Context::SharedPtr context_ptr)
{
  std::lock_guard<std::recursive_mutex> lock(notify_guard_condition_mutex_);
  if (notify_guard_condition_ && context_ptr != notify_guard_condition_->get_context()) {
    if (associated_with_executor_) {
      trigger_notify_guard_condition();
    }
    notify_guard_condition_ = nullptr;
  }

  if (!notify_guard_condition_) {
    notify_guard_condition_ = std::make_shared<rclcpp::GuardCondition>(context_ptr);
  }

  return notify_guard_condition_;
}

void
CallbackGroup::trigger_notify_guard_condition()
{
  std::lock_guard<std::recursive_mutex> lock(notify_guard_condition_mutex_);
  if (notify_guard_condition_) {
    notify_guard_condition_->trigger();
  }
}

void
CallbackGroup::add_subscription(
  const rclcpp::SubscriptionBase::SharedPtr subscription_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  subscription_ptrs_.push_back(subscription_ptr);
  subscription_ptrs_.erase(
    std::remove_if(
      subscription_ptrs_.begin(),
      subscription_ptrs_.end(),
      [](rclcpp::SubscriptionBase::WeakPtr x) {return x.expired();}),
    subscription_ptrs_.end());
}

void
CallbackGroup::add_timer(const rclcpp::TimerBase::SharedPtr timer_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timer_ptrs_.push_back(timer_ptr);
  timer_ptrs_.erase(
    std::remove_if(
      timer_ptrs_.begin(),
      timer_ptrs_.end(),
      [](rclcpp::TimerBase::WeakPtr x) {return x.expired();}),
    timer_ptrs_.end());
}

void
CallbackGroup::add_service(const rclcpp::ServiceBase::SharedPtr service_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  service_ptrs_.push_back(service_ptr);
  service_ptrs_.erase(
    std::remove_if(
      service_ptrs_.begin(),
      service_ptrs_.end(),
      [](rclcpp::ServiceBase::WeakPtr x) {return x.expired();}),
    service_ptrs_.end());
}

void
CallbackGroup::add_client(const rclcpp::ClientBase::SharedPtr client_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  client_ptrs_.push_back(client_ptr);
  client_ptrs_.erase(
    std::remove_if(
      client_ptrs_.begin(),
      client_ptrs_.end(),
      [](rclcpp::ClientBase::WeakPtr x) {return x.expired();}),
    client_ptrs_.end());
}

void
CallbackGroup::add_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  waitable_ptrs_.push_back(waitable_ptr);
  waitable_ptrs_.erase(
    std::remove_if(
      waitable_ptrs_.begin(),
      waitable_ptrs_.end(),
      [](rclcpp::Waitable::WeakPtr x) {return x.expired();}),
    waitable_ptrs_.end());
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
