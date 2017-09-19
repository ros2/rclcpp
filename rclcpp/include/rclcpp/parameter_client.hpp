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

#include <memory>
#include <string>
#include <utility>
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
#include "rclcpp/create_subscription.hpp"
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
  RCLCPP_SMART_PTR_DEFINITIONS(AsyncParametersClient)

  RCLCPP_PUBLIC
  AsyncParametersClient(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

  RCLCPP_PUBLIC
  AsyncParametersClient(
    const rclcpp::node::Node::SharedPtr node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

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

  template<
    typename CallbackT,
    typename Alloc = std::allocator<void>,
    typename SubscriptionT =
    rclcpp::subscription::Subscription<rcl_interfaces::msg::ParameterEvent, Alloc>>
  typename rclcpp::subscription::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(CallbackT && callback)
  {
    using rclcpp::message_memory_strategy::MessageMemoryStrategy;
    auto msg_mem_strat =
      MessageMemoryStrategy<rcl_interfaces::msg::ParameterEvent, Alloc>::create_default();

    return rclcpp::create_subscription<
      rcl_interfaces::msg::ParameterEvent, CallbackT, Alloc, SubscriptionT>(
      this->node_topics_interface_.get(),
      "parameter_events",
      std::forward<CallbackT>(callback),
      rmw_qos_profile_default,
      nullptr,  // group,
      false,  // ignore_local_publications,
      false,  // use_intra_process_comms_,
      msg_mem_strat,
      std::make_shared<Alloc>());
  }

  RCLCPP_PUBLIC
  bool
  service_is_ready() const;

  template<typename RatioT = std::milli>
  bool
  wait_for_service(
    std::chrono::duration<int64_t, RatioT> timeout = std::chrono::duration<int64_t, RatioT>(-1))
  {
    return wait_for_service_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

protected:
  RCLCPP_PUBLIC
  bool
  wait_for_service_nanoseconds(std::chrono::nanoseconds timeout);

private:
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface_;
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
  RCLCPP_SMART_PTR_DEFINITIONS(SyncParametersClient)

  RCLCPP_PUBLIC
  explicit SyncParametersClient(
    rclcpp::node::Node::SharedPtr node,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

  RCLCPP_PUBLIC
  SyncParametersClient(
    rclcpp::executor::Executor::SharedPtr executor,
    rclcpp::node::Node::SharedPtr node,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

  RCLCPP_PUBLIC
  std::vector<rclcpp::parameter::ParameterVariant>
  get_parameters(const std::vector<std::string> & parameter_names);

  RCLCPP_PUBLIC
  bool
  has_parameter(const std::string & parameter_name);

  template<typename T>
  T
  get_parameter_impl(
    const std::string & parameter_name, std::function<T()> parameter_not_found_handler)
  {
    std::vector<std::string> names;
    names.push_back(parameter_name);
    auto vars = get_parameters(names);
    if ((vars.size() != 1) || (vars[0].get_type() == rclcpp::parameter::PARAMETER_NOT_SET)) {
      return parameter_not_found_handler();
    } else {
      return static_cast<T>(vars[0].get_value<T>());
    }
  }

  template<typename T>
  T
  get_parameter(const std::string & parameter_name, const T & default_value)
  {
    // *INDENT-OFF*
    return get_parameter_impl(parameter_name,
      std::function<T()>([&default_value]() -> T {return default_value; }));
    // *INDENT-ON*
  }

  template<typename T>
  T
  get_parameter(const std::string & parameter_name)
  {
    // *INDENT-OFF*
    return get_parameter_impl(parameter_name,
      std::function<T()>([]() -> T {throw std::runtime_error("Parameter not set"); }));
    // *INDENT-ON*
  }

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

  RCLCPP_PUBLIC
  bool
  service_is_ready() const
  {
    return async_parameters_client_->service_is_ready();
  }

  template<typename RatioT = std::milli>
  bool
  wait_for_service(
    std::chrono::duration<int64_t, RatioT> timeout = std::chrono::duration<int64_t, RatioT>(-1))
  {
    return async_parameters_client_->wait_for_service(timeout);
  }

private:
  rclcpp::executor::Executor::SharedPtr executor_;
  rclcpp::node::Node::SharedPtr node_;
  AsyncParametersClient::SharedPtr async_parameters_client_;
};

}  // namespace parameter_client
}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_CLIENT_HPP_
