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

#ifndef RCLCPP__PARAMETER_CLIENT_HPP_
#define RCLCPP__PARAMETER_CLIENT_HPP_

#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

#include "rcl/parameter_client.h"

#include "rclcpp/executors.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp
{
namespace parameter_client
{

using NameVector = std::vector<std::string>;
using ParameterVector = std::vector<rclcpp::parameter::ParameterVariant>;
using ParameterTypeVector = std::vector<rclcpp::parameter::ParameterType>;
using SetParametersResultVector = std::vector<rcl_interfaces::msg::SetParametersResult>;

class AsyncParametersClient
{
public:

  RCLCPP_SMART_PTR_DEFINITIONS(AsyncParametersClient);

  RCLCPP_PUBLIC
  AsyncParametersClient(
    const rclcpp::node::Node::SharedPtr node,
    const std::string & remote_node_name = "");

  RCLCPP_PUBLIC
  std::shared_future<ParameterVector>
  get_parameters(
    const NameVector & names,
    std::function<void(std::shared_future<ParameterVector>)> callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<ParameterTypeVector>
  get_parameter_types(
    const NameVector & names,
    std::function<void(std::shared_future<ParameterTypeVector>)> callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<SetParametersResultVector>
  set_parameters(
    const ParameterVector & parameters,
    std::function<void(std::shared_future<SetParametersResultVector>)> callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<rcl_interfaces::msg::SetParametersResult>
  set_parameters_atomically(
    const ParameterVector & parameters,
    std::function<
      void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)
    > callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<rcl_interfaces::msg::ListParametersResult>
  list_parameters(
    const NameVector & prefixes,
    uint64_t depth,
    std::function<
      void(std::shared_future<rcl_interfaces::msg::ListParametersResult>)
    > callback = nullptr);

  template<typename CallbackT>
  typename rclcpp::subscription::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(CallbackT && callback)
  {
    return node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "parameter_events", std::forward<CallbackT>(callback), rmw_qos_profile_parameter_events);
  }

private:
  const rclcpp::node::Node::SharedPtr node_;
  rcl_parameter_client_t parameter_client_handle_ = rcl_get_zero_initialized_parameter_client();

#define RCLCPP_PARAMETER_CLIENT(REQUEST, RESPONSE) \
 rclcpp::client::ClientPattern<REQUEST, RESPONSE>

#define RCLCPP_PARAMETER_CLIENT_SRV(SERVICE_TYPE) \
 rclcpp::client::ClientPattern<SERVICE_TYPE::Request, SERVICE_TYPE::Response>


  // Storage for promise/future patterns
  RCLCPP_PARAMETER_CLIENT(NameVector, ParameterVector) get_parameters_client_;
  RCLCPP_PARAMETER_CLIENT(NameVector, ParameterTypeVector) get_parameter_types_client_;
  RCLCPP_PARAMETER_CLIENT(ParameterVector, SetParametersResultVector) set_parameters_client_;
  RCLCPP_PARAMETER_CLIENT(ParameterVector, rcl_interfaces::msg::SetParametersResult) set_parameters_atomically_client_;
  // TODO interface is a little strange
  RCLCPP_PARAMETER_CLIENT_SRV(rcl_interfaces::srv::ListParameters) list_parameters_client_;

  std::string remote_node_name_;
};

class SyncParametersClient
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SyncParametersClient);

  RCLCPP_PUBLIC
  SyncParametersClient(
    rclcpp::node::Node::SharedPtr node);

  RCLCPP_PUBLIC
  SyncParametersClient(
    rclcpp::executor::Executor::SharedPtr executor,
    rclcpp::node::Node::SharedPtr node);

  RCLCPP_PUBLIC
  ParameterVector
  get_parameters(const NameVector & parameter_names);

  RCLCPP_PUBLIC
  ParameterTypeVector
  get_parameter_types(const NameVector & parameter_names);

  RCLCPP_PUBLIC
  SetParametersResultVector
  set_parameters(const ParameterVector & parameters);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(const ParameterVector & parameters);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(
    const NameVector & parameter_prefixes,
    uint64_t depth);

  template<typename CallbackT>
  typename rclcpp::subscription::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(CallbackT && callback)
  {
    return async_parameters_client_->on_parameter_event(std::forward<CallbackT>(callback));
  }

private:
  rclcpp::executor::Executor::SharedPtr executor_;
  rclcpp::node::Node::SharedPtr node_;
  AsyncParametersClient::SharedPtr async_parameters_client_;
};

}  // namespace parameter_client
}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_CLIENT_HPP_
