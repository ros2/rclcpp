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
#include <vector>

#include <rmw/rmw.h>
#include <rosidl_generator_cpp/message_type_support.hpp>
#include <rosidl_generator_cpp/service_type_support.hpp>

#include <rclcpp/contexts/default_context.hpp>

#include <rcl_interfaces/GetParameters.h>
#include <rcl_interfaces/HasParameters.h>
#include <rcl_interfaces/Parameter.h>
#include <rcl_interfaces/ParameterDescription.h>
#include <rcl_interfaces/SetParameters.h>

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

template<typename ParamTypeT>
ParamTypeT
Node::get_parameter(const parameter::ParameterName & key) const
{
  const parameter::ParameterContainer pc(this->params_.at(key));
  return pc.get_value<ParamTypeT>();
}

template<typename ParamTypeT>
void
Node::set_parameter(const parameter::ParameterName & key, const ParamTypeT & value)
{
  parameter::ParameterContainer pc(key, value);
  params_[key] = pc;
}

bool
Node::has_parameter(const parameter::ParameterQuery & query) const
{
  const parameter::ParameterName key = query.get_name();
  return params_.find(key) != params_.end();
}

std::vector<parameter::ParameterContainer>
Node::get_parameters(const std::vector<parameter::ParameterQuery> & queries) const
{
  std::vector<parameter::ParameterContainer> result;

  for (auto & kv: params_) {
    if (std::any_of(queries.cbegin(), queries.cend(),
      [&kv](const parameter::ParameterQuery & query) {
      return kv.first == query.get_name();
    }))
    {
      result.push_back(kv.second);
    }
  }
  return result;
}

template<typename ParamTypeT>
std::shared_future<ParamTypeT>
Node::async_get_parameter(
  const std::string & node_name, const parameter::ParameterName & key,
  std::function<void(std::shared_future<ParamTypeT>)> callback)
{
  std::promise<ParamTypeT> promise_result;
  auto future_result = promise_result.get_future().share();
  std::vector<parameter::ParameterName> param_names = {{key}};

  this->async_get_parameters(
    node_name, param_names,
    [&promise_result, &future_result, &callback, &param_names](
      std::shared_future<std::vector<parameter::ParameterContainer>> cb_f) {
    promise_result.set_value(cb_f.get()[0].get_value<ParamTypeT>());
    if (callback != nullptr) {
      callback(future_result);
    }
  });

  return future_result;
}


std::shared_future<std::vector<parameter::ParameterContainer>>
Node::async_get_parameters(
  const std::string & node_name, const std::vector<parameter::ParameterName> & parameter_names,
  std::function<void(std::shared_future<std::vector<parameter::ParameterContainer>>)> callback)
{
  std::promise<std::vector<parameter::ParameterContainer>> promise_result;
  auto future_result = promise_result.get_future().share();
  if (node_name == this->get_name()) {
    std::vector<parameter::ParameterContainer> value;
    for (auto pn : parameter_names) {
      if (this->has_parameter(pn)) {
        try {
          parameter::ParameterContainer param_container(pn, this->get_parameter<int64_t>(pn));
          value.push_back(param_container);
          // TODO: use custom exception
        } catch (...) {
          try {
            parameter::ParameterContainer param_container(pn, this->get_parameter<double>(pn));
            value.push_back(param_container);
          } catch (...) {
            try {
              parameter::ParameterContainer param_container(pn,
                this->get_parameter<std::string>(pn));
              value.push_back(param_container);
            } catch (...) {
              parameter::ParameterContainer param_container(pn, this->get_parameter<bool>(pn));
              value.push_back(param_container);
            }
          }
        }
      }
    }
    promise_result.set_value(value);
    if (callback != nullptr) {
      callback(future_result);
    }
  } else {
    auto client = this->create_client<rcl_interfaces::GetParameters>("get_params");
    auto request = std::make_shared<rcl_interfaces::GetParameters::Request>();
    for (auto parameter_name : parameter_names) {
      rcl_interfaces::ParameterDescription description;
      description.name = parameter_name;
      request->parameter_descriptions.push_back(description);
    }

    client->async_send_request(
      request, [&promise_result, &future_result, &callback](
        rclcpp::client::Client<rcl_interfaces::GetParameters>::SharedFuture cb_f) {
      std::vector<parameter::ParameterContainer> value;
      auto parameters = cb_f.get()->parameters;
      for (auto parameter : parameters) {
        switch (parameter.description.parameter_type) {
          case rcl_interfaces::ParameterDescription::BOOL_PARAMETER:
            value.push_back(parameter::ParameterContainer(
              parameter.description.name,
              rclcpp::parameter::get_parameter_value<bool>(parameter)));
            break;
          case rcl_interfaces::ParameterDescription::STRING_PARAMETER:
            value.push_back(parameter::ParameterContainer(
              parameter.description.name,
              rclcpp::parameter::get_parameter_value<std::string>(parameter)));
            break;
          case rcl_interfaces::ParameterDescription::DOUBLE_PARAMETER:
            value.push_back(parameter::ParameterContainer(
              parameter.description.name,
              rclcpp::parameter::get_parameter_value<double>(parameter)));
            break;
          case rcl_interfaces::ParameterDescription::INTEGER_PARAMETER:
            value.push_back(parameter::ParameterContainer(
              parameter.description.name,
              rclcpp::parameter::get_parameter_value<int64_t>(parameter)));
            break;
          default:
            // TODO: use custom exception
            throw std::runtime_error("Invalid value type");
        }
      }
      promise_result.set_value(value);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
      );
  }
  return future_result;
}

std::shared_future<bool>
Node::async_set_parameters(
  const std::string & node_name,
  const std::vector<parameter::ParameterContainer> & key_values,
  std::function<void(std::shared_future<bool>)> callback)
{
  std::promise<bool> promise_result;
  auto future_result = promise_result.get_future().share();
  if (node_name == this->get_name()) {
    for (auto kv : key_values) {
      switch (kv.get_typeID()) {
        case parameter::ParameterDataType::INTEGER_PARAMETER:
          this->set_parameter(kv.get_name(), kv.get_value<int64_t>());
          break;
        case parameter::ParameterDataType::DOUBLE_PARAMETER:
          this->set_parameter(kv.get_name(), kv.get_value<double>());
          break;
        case parameter::ParameterDataType::STRING_PARAMETER:
          this->set_parameter(kv.get_name(), kv.get_value<std::string>());
          break;
        case parameter::ParameterDataType::BOOL_PARAMETER:
          this->set_parameter(kv.get_name(), kv.get_value<bool>());
          break;
        default:
          throw std::runtime_error("Invalid parameter type");
      }
    }
    promise_result.set_value(true);
    if (callback != nullptr) {
      callback(future_result);
    }
  } else {
    auto client = this->create_client<rcl_interfaces::SetParameters>("set_params");
    auto request = std::make_shared<rcl_interfaces::SetParameters::Request>();
    for (auto kv: key_values) {
      rcl_interfaces::Parameter parameter;
      parameter.description.name = kv.get_name();
      switch (kv.get_typeID()) {
        case parameter::ParameterDataType::INTEGER_PARAMETER:
          rclcpp::parameter::set_parameter_value<int64_t>(
            parameter, kv.get_value<int64_t>());
          break;
        case parameter::ParameterDataType::DOUBLE_PARAMETER:
          rclcpp::parameter::set_parameter_value<double>(
            parameter, kv.get_value<double>());
          break;
        case parameter::ParameterDataType::STRING_PARAMETER:
          rclcpp::parameter::set_parameter_value<std::string>(
            parameter, kv.get_value<std::string>());
          break;
        case parameter::ParameterDataType::BOOL_PARAMETER:
          rclcpp::parameter::set_parameter_value<bool>(
            parameter, kv.get_value<bool>());
          break;
        default:
          throw std::runtime_error("Invalid parameter type");
      }
      request->parameters.push_back(parameter);
    }

    client->async_send_request(
      request, [&promise_result, &future_result, &callback](
        rclcpp::client::Client<rcl_interfaces::SetParameters>::SharedFuture cb_f) {
      promise_result.set_value(cb_f.get()->success);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
      );
  }
  return future_result;
}

template<typename ParamTypeT>
std::shared_future<bool>
Node::async_set_parameter(
  const std::string & node_name, const parameter::ParameterName & key, const ParamTypeT & value,
  std::function<void(std::shared_future<bool>)> callback)
{
  std::vector<parameter::ParameterContainer> param_containers = {{
                                                                   parameter::ParameterContainer(
                                                                     key, value)
                                                                 }};
  return this->async_set_parameters(node_name, param_containers, callback);
}

std::shared_future<bool>
Node::async_has_parameter(
  const std::string & node_name, const parameter::ParameterQuery & query,
  std::function<void(std::shared_future<bool>)> callback)
{
  std::promise<bool> promise_result;
  auto future_result = promise_result.get_future().share();
  if (node_name == this->get_name()) {
    promise_result.set_value(this->has_parameter(query));
  } else {
    auto client = this->create_client<rcl_interfaces::HasParameters>("has_params");
    auto request = std::make_shared<rcl_interfaces::HasParameters::Request>();
    rcl_interfaces::ParameterDescription parameter_description;
    parameter_description.name = query.get_name();
    request->parameter_descriptions.push_back(parameter_description);
    client->async_send_request(
      request, [&promise_result, &future_result, &callback](
        rclcpp::client::Client<rcl_interfaces::HasParameters>::SharedFuture cb_f) {
      promise_result.set_value(cb_f.get()->result[0]);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
      );
  }
  return future_result;
}

#endif /* RCLCPP_RCLCPP_NODE_IMPL_HPP_ */
