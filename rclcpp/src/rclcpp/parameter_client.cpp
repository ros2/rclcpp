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

#include "rosidl_generator_c/string_functions.h"
#include "rcl_interfaces/msg/parameter.h"

#include "rclcpp/parameter_client.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

using rclcpp::parameter_client::AsyncParametersClient;
using rclcpp::parameter_client::SyncParametersClient;
using rclcpp::parameter_client::NameVector;
using rclcpp::parameter_client::ParameterVector;
using rclcpp::parameter_client::ParameterTypeVector;
using rclcpp::parameter_client::SetParametersResultVector;


// probably need to be in utilities.hpp
// Expects array to be initialized to the same size as str_vector
void convert_to_rosidl_generator_c_string(
  const std::vector<std::string> & input, rosidl_generator_c__String__Array & output)
{
  for (size_t i = 0; i < input.size(); ++i) {
    rosidl_generator_c__String__assign(&output.data[i], input[i].c_str());
  }
}

void convert_to_c_array(const std::vector<rclcpp::parameter::ParameterVariant> & input,
  rcl_interfaces__msg__Parameter__Array & output);
/*{
  
  for (size_t i = 0; i < input.size(); ++i) {
    // rosidl_generator_c__String__assign(&output.data[i], input[i].c_str());
    rosidl_generator_c__String__assign(output.data[i].name, input[i].get_name().c_str());
    output.data[i].value = ;
  }
}*/

AsyncParametersClient::AsyncParametersClient(
  const rclcpp::node::Node::SharedPtr node,
  const std::string & remote_node_name)
: node_(node)
{
  if (remote_node_name != "") {
    remote_node_name_ = remote_node_name;
  } else {
    remote_node_name_ = node_->get_name();
  }
  rcl_parameter_client_options_t options = rcl_parameter_client_get_default_options();
  memcpy(options.remote_node_name, remote_node_name.c_str(), remote_node_name.length());
  rcl_ret_t ret = rcl_parameter_client_init(&parameter_client_handle_, node->get_rcl_handle(), &options);
  if (ret != RCL_RET_OK) {
    // TODO error handling
  }

  // TODO move semantics for moving lambdas into the clients
  auto get_parameters_request_function = [this](const NameVector & names, int64_t & sequence_number) {
    rosidl_generator_c__String__Array names_array;
    rosidl_generator_c__String__Array__init(&names_array, names.size());
    convert_to_rosidl_generator_c_string(names, names_array);

    rcl_ret_t ret = rcl_parameter_client_send_get_request(
      &parameter_client_handle_, &names_array, &sequence_number);
  };
  get_parameters_client_.set_send_request_function(get_parameters_request_function);

  auto get_parameter_types_request_function = [this](const NameVector & names, int64_t & sequence_number) {
    rosidl_generator_c__String__Array names_array;
    rosidl_generator_c__String__Array__init(&names_array, names.size());
    convert_to_rosidl_generator_c_string(names, names_array);

    rcl_ret_t ret = rcl_parameter_client_send_get_types_request(
      &parameter_client_handle_, &names_array, &sequence_number);
  };
  get_parameter_types_client_.set_send_request_function(get_parameter_types_request_function);

  auto set_parameters_request_function = [this](const ParameterVector & params, int64_t & sequence_number) {
    rcl_interfaces__msg__Parameter__Array param_array;
    rcl_interfaces__msg__Parameter__Array__init(&param_array, params.size());
    convert_to_c_array(params, param_array);

    rcl_ret_t ret = rcl_parameter_client_send_set_request(
      &parameter_client_handle_, &param_array, &sequence_number);
  };
  set_parameters_client_.set_send_request_function(set_parameters_request_function);

}

std::shared_future<std::vector<rclcpp::parameter::ParameterVariant>>
AsyncParametersClient::get_parameters(
  const NameVector & names,
  std::function<void(std::shared_future<ParameterVector>)> callback)
{
  auto promise_result =
    std::make_shared<std::promise<std::vector<rclcpp::parameter::ParameterVariant>>>();
  auto future_result = promise_result->get_future().share();

  get_parameters_client_.async_send_request(
    names,
    [promise_result, future_result, &callback](
      RCLCPP_PARAMETER_CLIENT(NameVector, ParameterVector)::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get());
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

std::shared_future<std::vector<rclcpp::parameter::ParameterType>>
AsyncParametersClient::get_parameter_types(
  const NameVector & names,
  std::function<void(std::shared_future<ParameterTypeVector>)> callback)
{
  auto promise_result = std::make_shared<std::promise<ParameterTypeVector>>();
  auto future_result = promise_result->get_future().share();

  get_parameter_types_client_.async_send_request(
    names,
    [promise_result, future_result, &callback](
      RCLCPP_PARAMETER_CLIENT(NameVector, ParameterTypeVector)::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get());
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

std::shared_future<SetParametersResultVector>
AsyncParametersClient::set_parameters(
  const ParameterVector & parameters,
  std::function<void(std::shared_future<SetParametersResultVector>)> callback)
{
  auto promise_result = std::make_shared<std::promise<SetParametersResultVector>>();
  auto future_result = promise_result->get_future().share();

  set_parameters_client_.async_send_request(
    parameters,
    [promise_result, future_result, &callback](
      RCLCPP_PARAMETER_CLIENT(ParameterVector, SetParametersResultVector)::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get());
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );

  return future_result;
}

std::shared_future<rcl_interfaces::msg::SetParametersResult>
AsyncParametersClient::set_parameters_atomically(
  const ParameterVector & parameters,
  std::function<
    void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)
  > callback)
{
  auto promise_result =
    std::make_shared<std::promise<rcl_interfaces::msg::SetParametersResult>>();
  auto future_result = promise_result->get_future().share();

/*
  auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

  // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
  std::transform(parameters.begin(), parameters.end(), std::back_inserter(request->parameters),
    [](rclcpp::parameter::ParameterVariant p) {
      return p.to_parameter();
    }
  );

  set_parameters_atomically_client_->async_send_request(
    request,
    [promise_result, future_result, &callback](
      rclcpp::client::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get()->result);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );
  // *INDENT-ON*
*/

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

/*
  auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
  request->prefixes = prefixes;
  request->depth = depth;

  // *INDENT-OFF* (prevent uncrustify from making unecessary indents here)
  list_parameters_client_->async_send_request(
    request,
    [promise_result, future_result, &callback](
      rclcpp::client::Client<rcl_interfaces::srv::ListParameters>::SharedFuture cb_f)
    {
      promise_result->set_value(cb_f.get()->result);
      if (callback != nullptr) {
        callback(future_result);
      }
    }
  );
  // *INDENT-ON*
*/

  return future_result;
}

SyncParametersClient::SyncParametersClient(
  rclcpp::node::Node::SharedPtr node)
: node_(node)
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  async_parameters_client_ = std::make_shared<AsyncParametersClient>(node);
}

SyncParametersClient::SyncParametersClient(
  rclcpp::executor::Executor::SharedPtr executor,
  rclcpp::node::Node::SharedPtr node)
: executor_(executor), node_(node)
{
  async_parameters_client_ = std::make_shared<AsyncParametersClient>(node);
}

std::vector<rclcpp::parameter::ParameterVariant>
SyncParametersClient::get_parameters(const std::vector<std::string> & parameter_names)
{
  auto f = async_parameters_client_->get_parameters(parameter_names);
  if (rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }
  // Return an empty vector if unsuccessful
  return std::vector<rclcpp::parameter::ParameterVariant>();
}

std::vector<rclcpp::parameter::ParameterType>
SyncParametersClient::get_parameter_types(const std::vector<std::string> & parameter_names)
{
  auto f = async_parameters_client_->get_parameter_types(parameter_names);

  if (rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }
  return std::vector<rclcpp::parameter::ParameterType>();
}

std::vector<rcl_interfaces::msg::SetParametersResult>
SyncParametersClient::set_parameters(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  auto f = async_parameters_client_->set_parameters(parameters);

  if (rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }
  return std::vector<rcl_interfaces::msg::SetParametersResult>();
}

rcl_interfaces::msg::SetParametersResult
SyncParametersClient::set_parameters_atomically(
  const std::vector<rclcpp::parameter::ParameterVariant> & parameters)
{
  auto f = async_parameters_client_->set_parameters_atomically(parameters);

  if (rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }

  throw std::runtime_error("Unable to get result of set parameters service call.");
}

rcl_interfaces::msg::ListParametersResult
SyncParametersClient::list_parameters(
  const std::vector<std::string> & parameter_prefixes,
  uint64_t depth)
{
  auto f = async_parameters_client_->list_parameters(parameter_prefixes, depth);

  if (rclcpp::executors::spin_node_until_future_complete(*executor_, node_, f) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    return f.get();
  }

  throw std::runtime_error("Unable to get result of list parameters service call.");
}
