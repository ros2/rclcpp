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

#ifndef RCLCPP_RCLCPP_NODE_IMPL_HPP_
#define RCLCPP_RCLCPP_NODE_IMPL_HPP_

#include <algorithm>
#include <memory>
#include <string>

#include <rmw/rmw.h>
#include <rosidl_generator_cpp/message_type_support.hpp>
#include <rosidl_generator_cpp/service_type_support.hpp>

#include <rclcpp/contexts/default_context.hpp>

#ifndef RCLCPP_RCLCPP_NODE_HPP_
#include "node.hpp"
#endif

using namespace rclcpp;
using namespace rclcpp::node;

using rclcpp::contexts::default_context::DefaultContext;

Node::Node(std::string node_name)
: Node(node_name, DefaultContext::make_shared())
{}

Node::Node(std::string node_name, context::Context::SharedPtr context)
: name_(node_name), context_(context),
  number_of_subscriptions_(0), number_of_timers_(0), number_of_services_(0)
{
  node_handle_ = rmw_create_node(name_.c_str());
  using rclcpp::callback_group::CallbackGroupType;
  default_callback_group_ = \
    create_callback_group(CallbackGroupType::MutuallyExclusive);
}

rclcpp::callback_group::CallbackGroup::SharedPtr
Node::create_callback_group(
  rclcpp::callback_group::CallbackGroupType group_type)
{
  using rclcpp::callback_group::CallbackGroup;
  using rclcpp::callback_group::CallbackGroupType;
  auto group = CallbackGroup::SharedPtr(new CallbackGroup(group_type));
  callback_groups_.push_back(group);
  return group;
}

template<typename MessageT>
publisher::Publisher::SharedPtr
Node::create_publisher(std::string topic_name, size_t queue_size)
{
  using rosidl_generator_cpp::get_message_type_support_handle;
  auto type_support_handle = get_message_type_support_handle<MessageT>();
  rmw_publisher_t * publisher_handle = rmw_create_publisher(
    node_handle_, type_support_handle, topic_name.c_str(), queue_size);

  return publisher::Publisher::make_shared(publisher_handle);
}

bool
Node::group_in_node(callback_group::CallbackGroup::SharedPtr & group)
{
  bool group_belongs_to_this_node = false;
  for (auto & weak_group : this->callback_groups_) {
    auto cur_group = weak_group.lock();
    if (cur_group && (cur_group == group)) {
      group_belongs_to_this_node = true;
    }
  }
  return group_belongs_to_this_node;
}

template<typename MessageT>
typename subscription::Subscription<MessageT>::SharedPtr
Node::create_subscription(
  std::string topic_name,
  size_t queue_size,
  std::function<void(const std::shared_ptr<MessageT> &)> callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  using rosidl_generator_cpp::get_message_type_support_handle;
  auto type_support_handle = get_message_type_support_handle<MessageT>();
  rmw_subscription_t * subscriber_handle = rmw_create_subscription(
    node_handle_, type_support_handle, topic_name.c_str(), queue_size);

  using namespace rclcpp::subscription;

  auto sub = Subscription<MessageT>::make_shared(
    subscriber_handle,
    topic_name,
    callback);
  auto sub_base_ptr = std::dynamic_pointer_cast<SubscriptionBase>(sub);
  if (group) {
    if (!group_in_node(group)) {
      // TODO: use custom exception
      throw std::runtime_error("Cannot create timer, group not in node.");
    }
    group->add_subscription(sub_base_ptr);
  } else {
    default_callback_group_->add_subscription(sub_base_ptr);
  }
  number_of_subscriptions_++;
  return sub;
}

rclcpp::timer::WallTimer::SharedPtr
Node::create_wall_timer(
  std::chrono::nanoseconds period,
  rclcpp::timer::CallbackType callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  auto timer = rclcpp::timer::WallTimer::make_shared(period, callback);
  if (group) {
    if (!group_in_node(group)) {
      // TODO: use custom exception
      throw std::runtime_error("Cannot create timer, group not in node.");
    }
    group->add_timer(timer);
  } else {
    default_callback_group_->add_timer(timer);
  }
  number_of_timers_++;
  return timer;
}

rclcpp::timer::WallTimer::SharedPtr
Node::create_wall_timer(
  std::chrono::duration<long double, std::nano> period,
  rclcpp::timer::CallbackType callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  return create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    callback,
    group);
}

template<typename ServiceT>
typename client::Client<ServiceT>::SharedPtr
Node::create_client(
  const std::string & service_name,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  using rosidl_generator_cpp::get_service_type_support_handle;
  auto service_type_support_handle =
    get_service_type_support_handle<ServiceT>();

  rmw_client_t * client_handle = rmw_create_client(
    this->node_handle_, service_type_support_handle, service_name.c_str());

  using namespace rclcpp::client;

  auto cli = Client<ServiceT>::make_shared(
    client_handle,
    service_name);

  auto cli_base_ptr = std::dynamic_pointer_cast<ClientBase>(cli);
  if (group) {
    if (!group_in_node(group)) {
      // TODO(esteve): use custom exception
      throw std::runtime_error("Cannot create client, group not in node.");
    }
    group->add_client(cli_base_ptr);
  } else {
    default_callback_group_->add_client(cli_base_ptr);
  }
  number_of_clients_++;

  return cli;
}

template<typename ServiceT>
typename service::Service<ServiceT>::SharedPtr
Node::create_service(
  const std::string & service_name,
  typename rclcpp::service::Service<ServiceT>::CallbackType callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  using rosidl_generator_cpp::get_service_type_support_handle;
  auto service_type_support_handle =
    get_service_type_support_handle<ServiceT>();

  rmw_service_t * service_handle = rmw_create_service(
    this->node_handle_, service_type_support_handle, service_name.c_str());

  auto serv = service::Service<ServiceT>::make_shared(
    service_handle,
    service_name,
    callback);
  auto serv_base_ptr = std::dynamic_pointer_cast<service::ServiceBase>(serv);
  register_service(service_name, serv_base_ptr, group);
  return serv;
}

template<typename ServiceT>
typename service::Service<ServiceT>::SharedPtr
Node::create_service(
  const std::string & service_name,
  typename rclcpp::service::Service<ServiceT>::CallbackWithHeaderType callback_with_header,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  using rosidl_generator_cpp::get_service_type_support_handle;
  auto service_type_support_handle =
    get_service_type_support_handle<ServiceT>();

  rmw_service_t * service_handle = rmw_create_service(
    this->node_handle_, service_type_support_handle, service_name.c_str());

  auto serv = service::Service<ServiceT>::make_shared(
    service_handle,
    service_name,
    callback_with_header);
  auto serv_base_ptr = std::dynamic_pointer_cast<service::ServiceBase>(serv);
  register_service(service_name, serv_base_ptr, group);
  return serv;
}

void
Node::register_service(
  const std::string & service_name,
  std::shared_ptr<rclcpp::service::ServiceBase> serv_base_ptr,
  rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  if (group) {
    if (!group_in_node(group)) {
      // TODO: use custom exception
      throw std::runtime_error("Cannot create service, group not in node.");
    }
    group->add_service(serv_base_ptr);
  } else {
    default_callback_group_->add_service(serv_base_ptr);
  }
  number_of_services_++;
}

template <typename ParamTypeT>
ParamTypeT
Node::get_param(const parameter::ParamName& key) const
{
  const parameter::ParamContainer pc = this->params_.at(key);
  ParamTypeT value;
  return pc.get_value<ParamTypeT>(value);
}

template <typename ParamTypeT>
void
Node::set_param(const parameter::ParamName& key, const ParamTypeT& value)
{
  parameter::ParamContainer pc;
  pc.set_value(key, value);
  params_[key] = pc;
}

bool
Node::has_param(const parameter::ParamQuery& query) const
{
  const parameter::ParamName key = query.get_name();
  return (params_.find(key) != params_.end());
}

std::vector<parameter::ParamContainer>
Node::get_params(const std::vector<parameter::ParamQuery>& queries) const
{
  std::vector<parameter::ParamContainer> result;

  for(auto& kv: params_) {
    if(std::any_of(queries.cbegin(), queries.cend(),
      [&kv](const parameter::ParamQuery& query) {
      return kv.first == query.get_name();})) {
      result.push_back(kv.second);
    }
  }
  return result;
}
#endif /* RCLCPP_RCLCPP_NODE_IMPL_HPP_ */
