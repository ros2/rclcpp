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

class AsyncParametersClient
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AsyncParametersClient);

  RCLCPP_PUBLIC
  AsyncParametersClient(
    const rclcpp::node::Node::SharedPtr node,
    const std::string & remote_node_name = "");

  RCLCPP_PUBLIC
  std::shared_future<std::vector<rclcpp::parameter::ParameterVariant>>
  get_parameters(
    const std::vector<std::string> & names,
    std::function<
      void(std::shared_future<std::vector<rclcpp::parameter::ParameterVariant>>)
    > callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<std::vector<rclcpp::parameter::ParameterType>>
  get_parameter_types(
    const std::vector<std::string> & names,
    std::function<
      void(std::shared_future<std::vector<rclcpp::parameter::ParameterType>>)
    > callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
  set_parameters(
    const std::vector<rclcpp::parameter::ParameterVariant> & parameters,
    std::function<
      void(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>)
    > callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<rcl_interfaces::msg::SetParametersResult>
  set_parameters_atomically(
    const std::vector<rclcpp::parameter::ParameterVariant> & parameters,
    std::function<
      void(std::shared_future<rcl_interfaces::msg::SetParametersResult>)
    > callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<rcl_interfaces::msg::ListParametersResult>
  list_parameters(
    const std::vector<std::string> & prefixes,
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
  rclcpp::client::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
  rclcpp::client::Client<rcl_interfaces::srv::GetParameterTypes>::SharedPtr
    get_parameter_types_client_;
  rclcpp::client::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  rclcpp::client::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr
    set_parameters_atomically_client_;
  rclcpp::client::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
  rclcpp::client::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr
    describe_parameters_client_;
  std::string remote_node_name_;
};

class SyncParametersClient
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SyncParametersClient);

  RCLCPP_PUBLIC
  explicit SyncParametersClient(rclcpp::node::Node::SharedPtr node);

  RCLCPP_PUBLIC
  SyncParametersClient(
    rclcpp::executor::Executor::SharedPtr executor,
    rclcpp::node::Node::SharedPtr node);

  RCLCPP_PUBLIC
  std::vector<rclcpp::parameter::ParameterVariant>
  get_parameters(const std::vector<std::string> & parameter_names);

  RCLCPP_PUBLIC
  std::vector<rclcpp::parameter::ParameterType>
  get_parameter_types(const std::vector<std::string> & parameter_names);

  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(const std::vector<rclcpp::parameter::ParameterVariant> & parameters);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(const std::vector<rclcpp::parameter::ParameterVariant> & parameters);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(
    const std::vector<std::string> & parameter_prefixes,
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
