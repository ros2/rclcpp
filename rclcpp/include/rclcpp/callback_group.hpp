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

#ifndef RCLCPP__CALLBACK_GROUP_HPP_
#define RCLCPP__CALLBACK_GROUP_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/client.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{

// Forward declarations for friend statement in class CallbackGroup
namespace node_interfaces
{
class NodeServices;
class NodeTimers;
class NodeTopics;
class NodeWaitables;
}  // namespace node_interfaces

namespace callback_group
{

enum class CallbackGroupType
{
  MutuallyExclusive,
  Reentrant
};

class CallbackGroup
{
  friend class rclcpp::node_interfaces::NodeServices;
  friend class rclcpp::node_interfaces::NodeTimers;
  friend class rclcpp::node_interfaces::NodeTopics;
  friend class rclcpp::node_interfaces::NodeWaitables;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(CallbackGroup)

  RCLCPP_PUBLIC
  explicit CallbackGroup(CallbackGroupType group_type);

  RCLCPP_PUBLIC
  const std::vector<rclcpp::SubscriptionBase::WeakPtr> &
  get_subscription_ptrs() const;

  RCLCPP_PUBLIC
  const std::vector<rclcpp::TimerBase::WeakPtr> &
  get_timer_ptrs() const;

  RCLCPP_PUBLIC
  const std::vector<rclcpp::ServiceBase::WeakPtr> &
  get_service_ptrs() const;

  RCLCPP_PUBLIC
  const std::vector<rclcpp::ClientBase::WeakPtr> &
  get_client_ptrs() const;

  RCLCPP_PUBLIC
  const std::vector<rclcpp::Waitable::WeakPtr> &
  get_waitable_ptrs() const;

  RCLCPP_PUBLIC
  std::atomic_bool &
  can_be_taken_from();

  RCLCPP_PUBLIC
  const CallbackGroupType &
  type() const;

protected:
  RCLCPP_DISABLE_COPY(CallbackGroup)

  RCLCPP_PUBLIC
  void
  add_publisher(const rclcpp::PublisherBase::SharedPtr publisher_ptr);

  RCLCPP_PUBLIC
  void
  add_subscription(const rclcpp::SubscriptionBase::SharedPtr subscription_ptr);

  RCLCPP_PUBLIC
  void
  add_timer(const rclcpp::TimerBase::SharedPtr timer_ptr);

  RCLCPP_PUBLIC
  void
  add_service(const rclcpp::ServiceBase::SharedPtr service_ptr);

  RCLCPP_PUBLIC
  void
  add_client(const rclcpp::ClientBase::SharedPtr client_ptr);

  RCLCPP_PUBLIC
  void
  add_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr);

  RCLCPP_PUBLIC
  void
  remove_waitable(const rclcpp::Waitable::SharedPtr waitable_ptr) noexcept;

  CallbackGroupType type_;
  // Mutex to protect the subsequent vectors of pointers.
  mutable std::mutex mutex_;
  std::vector<rclcpp::SubscriptionBase::WeakPtr> subscription_ptrs_;
  std::vector<rclcpp::TimerBase::WeakPtr> timer_ptrs_;
  std::vector<rclcpp::ServiceBase::WeakPtr> service_ptrs_;
  std::vector<rclcpp::ClientBase::WeakPtr> client_ptrs_;
  std::vector<rclcpp::Waitable::WeakPtr> waitable_ptrs_;
  std::atomic_bool can_be_taken_from_;
};

}  // namespace callback_group
}  // namespace rclcpp

#endif  // RCLCPP__CALLBACK_GROUP_HPP_
