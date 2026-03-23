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
  const rclcpp::Context::WeakPtr & context,
  bool automatically_add_to_executor_with_node)
: type_(group_type), associated_with_executor_(false),
  can_be_taken_from_(true),
  automatically_add_to_executor_with_node_(automatically_add_to_executor_with_node),
  context_(context)
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

size_t
CallbackGroup::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return
    subscription_ptrs_.size() +
    service_ptrs_.size() +
    client_ptrs_.size() +
    timer_ptrs_.size() +
    waitable_ptrs_.size();
}

namespace
{
/// Iterate a vector of weak pointers, call func for each live entry,
/// and compact expired entries out of the vector in a single pass.
template<typename T, typename Func>
void collect_and_compact(
  std::vector<typename T::WeakPtr> & ptrs,
  const Func & func)
{
  size_t write_idx = 0;
  for (size_t read_idx = 0; read_idx < ptrs.size(); ++read_idx) {
    auto ref_ptr = ptrs[read_idx].lock();
    if (ref_ptr) {
      func(ref_ptr);
      if (write_idx != read_idx) {
        ptrs[write_idx] = std::move(ptrs[read_idx]);
      }
      ++write_idx;
    }
  }
  ptrs.resize(write_idx);
}
}  // namespace

void CallbackGroup::collect_all_ptrs(
  const std::function<void(const rclcpp::SubscriptionBase::SharedPtr &)> & sub_func,
  const std::function<void(const rclcpp::ServiceBase::SharedPtr &)> & service_func,
  const std::function<void(const rclcpp::ClientBase::SharedPtr &)> & client_func,
  const std::function<void(const rclcpp::TimerBase::SharedPtr &)> & timer_func,
  const std::function<void(const rclcpp::Waitable::SharedPtr &)> & waitable_func) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  collect_and_compact<rclcpp::SubscriptionBase>(subscription_ptrs_, sub_func);
  collect_and_compact<rclcpp::ServiceBase>(service_ptrs_, service_func);
  collect_and_compact<rclcpp::ClientBase>(client_ptrs_, client_func);
  collect_and_compact<rclcpp::TimerBase>(timer_ptrs_, timer_func);
  collect_and_compact<rclcpp::Waitable>(waitable_ptrs_, waitable_func);
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
CallbackGroup::get_notify_guard_condition()
{
  std::lock_guard<std::recursive_mutex> lock(notify_guard_condition_mutex_);
  rclcpp::Context::SharedPtr context_ptr = context_.lock();
  if (context_ptr && context_ptr->is_valid()) {
    if (!notify_guard_condition_) {
      notify_guard_condition_ = std::make_shared<rclcpp::GuardCondition>(context_ptr);
    }
    return notify_guard_condition_;
  }
  return nullptr;
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
  const rclcpp::SubscriptionBase::SharedPtr & subscription_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  subscription_ptrs_.push_back(subscription_ptr);
}

void
CallbackGroup::add_timer(const rclcpp::TimerBase::SharedPtr & timer_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timer_ptrs_.push_back(timer_ptr);
}

void
CallbackGroup::add_service(const rclcpp::ServiceBase::SharedPtr & service_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  service_ptrs_.push_back(service_ptr);
}

void
CallbackGroup::add_client(const rclcpp::ClientBase::SharedPtr & client_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  client_ptrs_.push_back(client_ptr);
}

void
CallbackGroup::add_waitable(const rclcpp::Waitable::SharedPtr & waitable_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  waitable_ptrs_.push_back(waitable_ptr);
}

void
CallbackGroup::remove_waitable(const rclcpp::Waitable::SharedPtr & waitable_ptr) noexcept
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
