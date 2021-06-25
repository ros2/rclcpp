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

#include "rclcpp/parameter_client.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "./parameter_service_names.hpp"

using rclcpp::AsyncParametersClient;
using rclcpp::SyncParametersClient;

AsyncParametersClient::AsyncParametersClient(
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
  const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
  const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
  const std::string & remote_node_name,
  const rmw_qos_profile_t & qos_profile,
  rclcpp::CallbackGroup::SharedPtr group)
: node_topics_interface_(node_topics_interface)
{
  if (remote_node_name != "") {
    remote_node_name_ = remote_node_name;
  } else {
    remote_node_name_ = node_base_interface->get_fully_qualified_name();
  }

  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos_profile;

  using rclcpp::Client;
  using rclcpp::ClientBase;

  get_parameters_client_ = Client<rcl_interfaces::srv::GetParameters>::make_shared(
    node_base_interface.get(),
    node_graph_interface,
    remote_node_name_ + "/" + parameter_service_names::get_parameters,
    options);
  auto get_parameters_base = std::dynamic_pointer_cast<ClientBase>(get_parameters_client_);
  node_services_interface->add_client(get_parameters_base, group);

  get_parameter_types_client_ = Client<rcl_interfaces::srv::GetParameterTypes>::make_shared(
    node_base_interface.get(),
    node_graph_interface,
    remote_node_name_ + "/" + parameter_service_names::get_parameter_types,
    options);
  auto get_parameter_types_base =
    std::dynamic_pointer_cast<ClientBase>(get_parameter_types_client_);
  node_services_interface->add_client(get_parameter_types_base, group);

  set_parameters_client_ = Client<rcl_interfaces::srv::SetParameters>::make_shared(
    node_base_interface.get(),
    node_graph_interface,
    remote_node_name_ + "/" + parameter_service_names::set_parameters,
    options);
  auto set_parameters_base = std::dynamic_pointer_cast<ClientBase>(set_parameters_client_);
  node_services_interface->add_client(set_parameters_base, group);

  set_parameters_atomically_client_ =
    Client<rcl_interfaces::srv::SetParametersAtomically>::make_shared(
    node_base_interface.get(),
    node_graph_interface,
    remote_node_name_ + "/" + parameter_service_names::set_parameters_atomically,
    options);
  auto set_parameters_atomically_base = std::dynamic_pointer_cast<ClientBase>(
    set_parameters_atomically_client_);
  node_services_interface->add_client(set_parameters_atomically_base, group);

  list_parameters_client_ = Client<rcl_interfaces::srv::ListParameters>::make_shared(
    node_base_interface.get(),
    node_graph_interface,
    remote_node_name_ + "/" + parameter_service_names::list_parameters,
    options);
  auto list_parameters_base = std::dynamic_pointer_cast<ClientBase>(list_parameters_client_);
  node_services_interface->add_client(list_parameters_base, group);

  describe_parameters_client_ = Client<rcl_interfaces::srv::DescribeParameters>::make_shared(
    node_base_interface.get(),
    node_graph_interface,
    remote_node_name_ + "/" + parameter_service_names::describe_parameters,
    options);
  auto describe_parameters_base =
    std::dynamic_pointer_cast<ClientBase>(describe_parameters_client_);
  node_services_interface->add_client(describe_parameters_base, group);
}

std::shared_future<std::vector<rclcpp::Parameter>>
AsyncParametersClient::get_parameters(
  const std::vector<std::string> & names,
  std::function<
    void(std::shared_future<std::vector<rclcpp::Parameter>>)
  > callback)
{
  auto promise_result =
    std::make_shared<std::promise<std::vector<rclcpp::Parameter>>>();
  auto future_result = promise_result->get_future().share();

  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = names;

  get_parameters_client_->async_send_request(
    request,
    [request, promise_result, future_result, callback](
      rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture cb_f)
    {
      std::vector<rclcpp::Parameter> parameters;
      auto & pvalues = cb_f.get()->values;

      for (auto & pvalue : pvalues) {
        auto i = static_cast<size_t>(&pvalue - &pvalues[0]);
        rcl_interfaces::msg::Parameter parameter;
        parameter.name = request->names[i];
        parameter.value = pvalue;
        parameters.push_back(rclcpp::Parameter::from_parameter_msg(parameter));
      }

      promise_result->set_value(parameters);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>
AsyncParametersClient::describe_parameters(
  const std::vector<std::string> & names,
  std::function<
    void(std::shared_future<std::vector<rcl_interfaces::msg::ParameterDescriptor>>)
  > callback)
{
  auto promise_result =
    std::make_shared<std::promise<std::vector<rcl_interfaces::msg::ParameterDescriptor>>>();
  auto future_result = promise_result->get_future().share();

  auto request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
  request->names = names;

  describe_parameters_client_->async_send_request(
    request,
    [promise_result, future_result, callback](
      rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get()->descriptors);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

std::shared_future<std::vector<rclcpp::ParameterType>>
AsyncParametersClient::get_parameter_types(
  const std::vector<std::string> & names,
  std::function<
    void(std::shared_future<std::vector<rclcpp::ParameterType>>)
  > callback)
{
  auto promise_result =
    std::make_shared<std::promise<std::vector<rclcpp::ParameterType>>>();
  auto future_result = promise_result->get_future().share();

  auto request = std::make_shared<rcl_interfaces::srv::GetParameterTypes::Request>();
  request->names = names;

  get_parameter_types_client_->async_send_request(
    request,
    [promise_result, future_result, callback](
      rclcpp::Client<rcl_interfaces::srv::GetParameterTypes>::SharedFuture cb_f)
    {
      std::vector<rclcpp::ParameterType> types;
      auto & pts = cb_f.get()->types;
      for (auto & pt : pts) {
        types.push_back(static_cast<rclcpp::ParameterType>(pt));
      }
      promise_result->set_value(types);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters,
  std::function<
    void(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>)
  > callback)
{
  auto promise_result =
    std::make_shared<std::promise<std::vector<rcl_interfaces::msg::SetParametersResult>>>();
  auto future_result = promise_result->get_future().share();

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

  std::transform(
    parameters.begin(), parameters.end(), std::back_inserter(request->parameters),
    [](rclcpp::Parameter p) {return p.to_parameter_msg();}
  );

  set_parameters_client_->async_send_request(
    request,
    [promise_result, future_result, callback](
      rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get()->results);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

std::shared_future<rcl_interfaces::msg::SetParametersResult>
AsyncParametersClient::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters,
  std::function<
    void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)
  > callback)
{
  auto promise_result =
    std::make_shared<std::promise<rcl_interfaces::msg::SetParametersResult>>();
  auto future_result = promise_result->get_future().share();

  auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

  std::transform(
    parameters.begin(), parameters.end(), std::back_inserter(request->parameters),
    [](rclcpp::Parameter p) {return p.to_parameter_msg();}
  );

  set_parameters_atomically_client_->async_send_request(
    request,
    [promise_result, future_result, callback](
      rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get()->result);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::delete_parameters(
  const std::vector<std::string> & parameters_names)
{
  std::vector<rclcpp::Parameter> parameters;
  for (const std::string & name : parameters_names) {
    parameters.push_back(rclcpp::Parameter(name));
  }
  auto future_result = set_parameters(parameters);

  return future_result;
}

std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::load_parameters(
  const std::string & yaml_filename)
{
  rclcpp::ParameterMap parameter_map = rclcpp::parameter_map_from_yaml_file(yaml_filename);
  return this->load_parameters(parameter_map);
}

std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
AsyncParametersClient::load_parameters(
  const rclcpp::ParameterMap & parameter_map)
{
  std::vector<rclcpp::Parameter> parameters;
  std::string remote_name = remote_node_name_.substr(remote_node_name_.substr(1).find("/") + 2);
  for (const auto & params : parameter_map) {
    std::string node_full_name = params.first;
    std::string node_name = node_full_name.substr(node_full_name.find("/*/") + 3);
    if (node_full_name == remote_node_name_ ||
      node_full_name == "/**" ||
      (node_name == remote_name))
    {
      for (const auto & param : params.second) {
        parameters.push_back(param.second.first);
      }
    }
  }

  if (parameters.size() == 0) {
    throw rclcpp::exceptions::InvalidParametersException("No valid parameter");
  }
  auto future_result = set_parameters(parameters);

  return future_result;
}


std::shared_future<rcl_interfaces::msg::ListParametersResult>
AsyncParametersClient::list_parameters(
  const std::vector<std::string> & prefixes,
  uint64_t depth,
  std::function<
    void(std::shared_future<rcl_interfaces::msg::ListParametersResult>)
  > callback)
{
  auto promise_result =
    std::make_shared<std::promise<rcl_interfaces::msg::ListParametersResult>>();
  auto future_result = promise_result->get_future().share();

  auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
  request->prefixes = prefixes;
  request->depth = depth;

  list_parameters_client_->async_send_request(
    request,
    [promise_result, future_result, callback](
      rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get()->result);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

bool
AsyncParametersClient::service_is_ready() const
{
  return
    get_parameters_client_->service_is_ready() &&
    get_parameter_types_client_->service_is_ready() &&
    set_parameters_client_->service_is_ready() &&
    list_parameters_client_->service_is_ready() &&
    describe_parameters_client_->service_is_ready();
}

bool
AsyncParametersClient::wait_for_service_nanoseconds(std::chrono::nanoseconds timeout)
{
  const std::vector<std::shared_ptr<rclcpp::ClientBase>> clients = {
    get_parameters_client_,
    get_parameter_types_client_,
    set_parameters_client_,
    list_parameters_client_,
    describe_parameters_client_
  };
  for (auto & client : clients) {
    auto stamp = std::chrono::steady_clock::now();
    if (!client->wait_for_service(timeout)) {
      return false;
    }
    if (timeout > std::chrono::nanoseconds::zero()) {
      timeout -= std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - stamp);
      if (timeout < std::chrono::nanoseconds::zero()) {
        timeout = std::chrono::nanoseconds::zero();
      }
    }
  }
  return true;
}

std::vector<rclcpp::Parameter>
SyncParametersClient::get_parameters(
  const std::vector<std::string> & parameter_names,
  std::chrono::nanoseconds timeout)
{
  auto f = async_parameters_client_->get_parameters(parameter_names);
  using rclcpp::executors::spin_node_until_future_complete;
  if (
    spin_node_until_future_complete(
      *executor_, node_base_interface_, f,
      timeout) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }
  // Return an empty vector if unsuccessful
  return std::vector<rclcpp::Parameter>();
}

bool
SyncParametersClient::has_parameter(const std::string & parameter_name)
{
  std::vector<std::string> names;
  names.push_back(parameter_name);
  auto vars = list_parameters(names, 1);
  return vars.names.size() > 0;
}

std::vector<rcl_interfaces::msg::ParameterDescriptor>
SyncParametersClient::describe_parameters(
  const std::vector<std::string> & parameter_names,
  std::chrono::nanoseconds timeout)
{
  auto f = async_parameters_client_->describe_parameters(parameter_names);

  using rclcpp::executors::spin_node_until_future_complete;
  rclcpp::FutureReturnCode future =
    spin_node_until_future_complete(*executor_, node_base_interface_, f, timeout);
  if (future == rclcpp::FutureReturnCode::SUCCESS) {
    return f.get();
  }
  return std::vector<rcl_interfaces::msg::ParameterDescriptor>();
}

std::vector<rclcpp::ParameterType>
SyncParametersClient::get_parameter_types(
  const std::vector<std::string> & parameter_names,
  std::chrono::nanoseconds timeout)
{
  auto f = async_parameters_client_->get_parameter_types(parameter_names);

  using rclcpp::executors::spin_node_until_future_complete;
  if (
    spin_node_until_future_complete(
      *executor_, node_base_interface_, f,
      timeout) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }
  return std::vector<rclcpp::ParameterType>();
}

std::vector<rcl_interfaces::msg::SetParametersResult>
SyncParametersClient::set_parameters(
  const std::vector<rclcpp::Parameter> & parameters,
  std::chrono::nanoseconds timeout)
{
  auto f = async_parameters_client_->set_parameters(parameters);

  using rclcpp::executors::spin_node_until_future_complete;
  if (
    spin_node_until_future_complete(
      *executor_, node_base_interface_, f,
      timeout) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }
  return std::vector<rcl_interfaces::msg::SetParametersResult>();
}

std::vector<rcl_interfaces::msg::SetParametersResult>
SyncParametersClient::delete_parameters(
  const std::vector<std::string> & parameters_names,
  std::chrono::nanoseconds timeout)
{
  auto f = async_parameters_client_->delete_parameters(parameters_names);

  using rclcpp::executors::spin_node_until_future_complete;
  if (
    spin_node_until_future_complete(
      *executor_, node_base_interface_, f,
      timeout) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }
  return std::vector<rcl_interfaces::msg::SetParametersResult>();
}

std::vector<rcl_interfaces::msg::SetParametersResult>
SyncParametersClient::load_parameters(
  const std::string & yaml_filename,
  std::chrono::nanoseconds timeout)
{
  auto f = async_parameters_client_->load_parameters(yaml_filename);

  using rclcpp::executors::spin_node_until_future_complete;
  if (
    spin_node_until_future_complete(
      *executor_, node_base_interface_, f,
      timeout) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }
  return std::vector<rcl_interfaces::msg::SetParametersResult>();
}

rcl_interfaces::msg::SetParametersResult
SyncParametersClient::set_parameters_atomically(
  const std::vector<rclcpp::Parameter> & parameters,
  std::chrono::nanoseconds timeout)
{
  auto f = async_parameters_client_->set_parameters_atomically(parameters);

  using rclcpp::executors::spin_node_until_future_complete;
  if (
    spin_node_until_future_complete(
      *executor_, node_base_interface_, f,
      timeout) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }

  throw std::runtime_error("Unable to get result of set parameters service call.");
}

rcl_interfaces::msg::ListParametersResult
SyncParametersClient::list_parameters(
  const std::vector<std::string> & parameter_prefixes,
  uint64_t depth,
  std::chrono::nanoseconds timeout)
{
  auto f = async_parameters_client_->list_parameters(parameter_prefixes, depth);

  using rclcpp::executors::spin_node_until_future_complete;
  if (
    spin_node_until_future_complete(
      *executor_, node_base_interface_, f,
      timeout) == rclcpp::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }

  throw std::runtime_error("Unable to get result of list parameters service call.");
}
