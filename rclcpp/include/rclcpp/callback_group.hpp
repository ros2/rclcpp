// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_RCLCPP_CALLBACK_GROUP_HPP_
#define RCLCPP_RCLCPP_CALLBACK_GROUP_HPP_

#include <atomic>
#include <string>
#include <vector>

#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/client.hpp>

namespace rclcpp
{

// Forward declarations for friend statement in class CallbackGroup
namespace node
{
class Node;
} // namespace node

namespace callback_group
{

enum class CallbackGroupType
{
  MutuallyExclusive,
  Reentrant
};

class CallbackGroup
{
  friend class rclcpp::node::Node;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(CallbackGroup);

  CallbackGroup(CallbackGroupType group_type)
  : type_(group_type), can_be_taken_from_(true)
  {}

  const std::vector<subscription::SubscriptionBase::WeakPtr> &
  get_subscription_ptrs() const
  {
    return subscription_ptrs_;
  }

  const std::vector<timer::TimerBase::WeakPtr> &
  get_timer_ptrs() const
  {
    return timer_ptrs_;
  }

  const std::vector<service::ServiceBase::SharedPtr> &
  get_service_ptrs() const
  {
    return service_ptrs_;
  }

  const std::vector<client::ClientBase::SharedPtr> &
  get_client_ptrs() const
  {
    return client_ptrs_;
  }

  std::atomic_bool & can_be_taken_from()
  {
    return can_be_taken_from_;
  }

  const CallbackGroupType & type() const
  {
    return type_;
  }

private:
  RCLCPP_DISABLE_COPY(CallbackGroup);

  void
  add_subscription(
    const subscription::SubscriptionBase::SharedPtr subscription_ptr)
  {
    subscription_ptrs_.push_back(subscription_ptr);
  }

  void
  add_timer(const timer::TimerBase::SharedPtr timer_ptr)
  {
    timer_ptrs_.push_back(timer_ptr);
  }

  void
  add_service(const service::ServiceBase::SharedPtr service_ptr)
  {
    service_ptrs_.push_back(service_ptr);
  }

  void
  add_client(const client::ClientBase::SharedPtr client_ptr)
  {
    client_ptrs_.push_back(client_ptr);
  }

  CallbackGroupType type_;
  std::vector<subscription::SubscriptionBase::WeakPtr> subscription_ptrs_;
  std::vector<timer::TimerBase::WeakPtr> timer_ptrs_;
  std::vector<service::ServiceBase::SharedPtr> service_ptrs_;
  std::vector<client::ClientBase::SharedPtr> client_ptrs_;
  std::atomic_bool can_be_taken_from_;

};

} /* namespace callback_group */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_CALLBACK_GROUP_HPP_ */
