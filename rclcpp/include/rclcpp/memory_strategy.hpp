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

#ifndef RCLCPP__MEMORY_STRATEGY_HPP_
#define RCLCPP__MEMORY_STRATEGY_HPP_

#include <memory>
#include <vector>

#include "rclcpp/any_executable.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace memory_strategy
{

/// Delegate for handling memory allocations while the Executor is executing.
/**
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */
class RCLCPP_PUBLIC MemoryStrategy
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(MemoryStrategy);
  using WeakNodeVector = std::vector<std::weak_ptr<rclcpp::node::Node>>;

  // return the new number of subscribers
  virtual size_t fill_subscriber_handles(void ** & ptr) = 0;

  // return the new number of services
  virtual size_t fill_service_handles(void ** & ptr) = 0;

  // return the new number of clients
  virtual size_t fill_client_handles(void ** & ptr) = 0;

  virtual void clear_active_entities() = 0;

  virtual void clear_handles() = 0;
  virtual void remove_null_handles() = 0;
  virtual bool collect_entities(const WeakNodeVector & weak_nodes) = 0;

  /// Provide a newly initialized AnyExecutable object.
  // \return Shared pointer to the fresh executable.
  virtual rclcpp::executor::AnyExecutable::SharedPtr instantiate_next_executable() = 0;

  virtual void
  get_next_subscription(rclcpp::executor::AnyExecutable::SharedPtr any_exec,
    const WeakNodeVector & weak_nodes) = 0;

  virtual void
  get_next_service(rclcpp::executor::AnyExecutable::SharedPtr any_exec,
    const WeakNodeVector & weak_nodes) = 0;

  virtual void
  get_next_client(rclcpp::executor::AnyExecutable::SharedPtr any_exec,
    const WeakNodeVector & weak_nodes) = 0;

  static rclcpp::subscription::SubscriptionBase::SharedPtr
  get_subscription_by_handle(void * subscriber_handle,
    const WeakNodeVector & weak_nodes);

  static rclcpp::service::ServiceBase::SharedPtr
  get_service_by_handle(void * service_handle, const WeakNodeVector & weak_nodes);

  static rclcpp::client::ClientBase::SharedPtr
  get_client_by_handle(void * client_handle, const WeakNodeVector & weak_nodes);

  static rclcpp::node::Node::SharedPtr
  get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr group,
    const WeakNodeVector & weak_nodes);

  static rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr subscription,
    const WeakNodeVector & weak_nodes);

  static rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_service(
    rclcpp::service::ServiceBase::SharedPtr service,
    const WeakNodeVector & weak_nodes);

  static rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_client(rclcpp::client::ClientBase::SharedPtr client,
    const WeakNodeVector & weak_nodes);
};

}  // namespace memory_strategy
}  // namespace rclcpp

#endif  // RCLCPP__MEMORY_STRATEGY_HPP_
