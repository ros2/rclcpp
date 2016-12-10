// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/intra_process_manager.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/type_support_decl.hpp"

#include "lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rclcpp_lifecycle
{

template<typename MessageT, typename Alloc>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, Alloc>>
LifecycleNode::create_publisher(
  const std::string & topic_name, size_t qos_history_depth,
  std::shared_ptr<Alloc> allocator)
{
  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = qos_history_depth;
  return this->create_publisher<MessageT, Alloc>(topic_name, qos, allocator);
}

template<typename MessageT, typename Alloc>
std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, Alloc>>
LifecycleNode::create_publisher(
  const std::string & topic_name,
  const rmw_qos_profile_t & qos_profile,
  std::shared_ptr<Alloc> allocator)
{
  using PublisherT = rclcpp_lifecycle::LifecyclePublisher<MessageT, Alloc>;

  // create regular publisher in rclcpp::Node
  return rclcpp::create_publisher<MessageT, Alloc, PublisherT>(
    this->node_topics_.get(),
    topic_name,
    qos_profile,
    use_intra_process_comms_,
    allocator);
}

// TODO(karsten1987): Create LifecycleSubscriber
template<typename MessageT, typename CallbackT, typename Alloc, typename SubscriptionT>
std::shared_ptr<SubscriptionT>
LifecycleNode::create_subscription(
  const std::string & topic_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
  msg_mem_strat,
  std::shared_ptr<Alloc> allocator)
{
  if (!allocator) {
    allocator = std::make_shared<Alloc>();
  }

  if (!msg_mem_strat) {
    using rclcpp::message_memory_strategy::MessageMemoryStrategy;
    msg_mem_strat = MessageMemoryStrategy<MessageT, Alloc>::create_default();
  }

  return rclcpp::create_subscription<
    MessageT, CallbackT, Alloc,
    rclcpp::subscription::Subscription<MessageT, Alloc>>(
    this->node_topics_.get(),
    topic_name,
    std::forward<CallbackT>(callback),
    qos_profile,
    group,
    ignore_local_publications,
    use_intra_process_comms_,
    msg_mem_strat,
    allocator);
}

template<
  typename MessageT,
  typename CallbackT,
  typename Alloc,
  typename SubscriptionT>
std::shared_ptr<SubscriptionT>
LifecycleNode::create_subscription(
  const std::string & topic_name,
  size_t qos_history_depth,
  CallbackT && callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group,
  bool ignore_local_publications,
  typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT, Alloc>::SharedPtr
  msg_mem_strat,
  std::shared_ptr<Alloc> allocator)
{
  rmw_qos_profile_t qos = rmw_qos_profile_default;
  qos.depth = qos_history_depth;
  return this->create_subscription<MessageT, CallbackT, Alloc>(
    topic_name,
    std::forward<CallbackT>(callback),
    qos,
    group,
    ignore_local_publications,
    msg_mem_strat,
    allocator);
}

template<typename DurationT, typename CallbackT>
typename rclcpp::timer::WallTimer<CallbackT>::SharedPtr
LifecycleNode::create_wall_timer(
  std::chrono::duration<int64_t, DurationT> period,
  CallbackT callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  auto timer = rclcpp::timer::WallTimer<CallbackT>::make_shared(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::move(callback));
  node_timers_->add_timer(timer, group);
  return timer;
}

template<typename ServiceT>
typename rclcpp::client::Client<ServiceT>::SharedPtr
LifecycleNode::create_client(
  const std::string & service_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos_profile;

  using rclcpp::client::Client;
  using rclcpp::client::ClientBase;

  auto cli = Client<ServiceT>::make_shared(
    node_base_.get(),
    node_graph_,
    service_name,
    options);

  auto cli_base_ptr = std::dynamic_pointer_cast<ClientBase>(cli);
  node_services_->add_client(cli_base_ptr, group);
  return cli;
}

template<typename ServiceT, typename CallbackT>
typename rclcpp::service::Service<ServiceT>::SharedPtr
LifecycleNode::create_service(
  const std::string & service_name,
  CallbackT && callback,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  rclcpp::service::AnyServiceCallback<ServiceT> any_service_callback;
  any_service_callback.set(std::forward<CallbackT>(callback));

  rcl_service_options_t service_options = rcl_service_get_default_options();
  service_options.qos = qos_profile;

  auto serv = rclcpp::service::Service<ServiceT>::make_shared(
    node_base_->get_shared_rcl_node_handle(),
    service_name, any_service_callback, service_options);
  auto serv_base_ptr = std::dynamic_pointer_cast<rclcpp::service::ServiceBase>(serv);
  node_services_->add_service(serv_base_ptr, group);
  return serv;
}

template<typename ParameterT>
bool
LifecycleNode::get_parameter(const std::string & name, ParameterT & parameter) const
{
  rclcpp::parameter::ParameterVariant parameter_variant(name, parameter);
  bool result = get_parameter(name, parameter_variant);
  parameter = parameter_variant.get_value<ParameterT>();

  return result;
}

template<typename CallbackT>
void
LifecycleNode::register_param_change_callback(CallbackT && callback)
{
  this->node_parameters_->register_param_change_callback(std::forward<CallbackT>(callback));
}

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_IMPL_HPP_
