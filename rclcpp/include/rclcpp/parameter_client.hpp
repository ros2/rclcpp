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

class AsyncParametersClient
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AsyncParametersClient)

  /// Create an async parameters client.
  /**
   * \param[in] node_base_interface The node base interface of the corresponding node.
   * \param[in] node_topics_interface Node topic base interface.
   * \param[in] node_graph_interface The node graph interface of the corresponding node.
   * \param[in] node_services_interface Node service interface.
   * \param[in] remote_node_name (optional) name of the remote node
   * \param[in] qos_profile (optional) The rmw qos profile to use to subscribe
   * \param[in] group (optional) The async parameter client will be added to this callback group.
   */
  RCLCPP_PUBLIC
  AsyncParametersClient(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// Constructor
  /**
   * \param[in] node The async paramters client will be added to this node.
   * \param[in] remote_node_name (optional) name of the remote node
   * \param[in] qos_profile (optional) The rmw qos profile to use to subscribe
   * \param[in] group (optional) The async parameter client will be added to this callback group.
   */
  RCLCPP_PUBLIC
  AsyncParametersClient(
    const rclcpp::Node::SharedPtr node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  /// Constructor
  /**
   * \param[in] node The  async paramters client will be added to this node.
   * \param[in] remote_node_name (optional) name of the remote node
   * \param[in] qos_profile (optional) The rmw qos profile to use to subscribe
   * \param[in] group (optional) The async parameter client will be added to this callback group.
   */
  RCLCPP_PUBLIC
  AsyncParametersClient(
    rclcpp::Node * node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters,
    rclcpp::CallbackGroup::SharedPtr group = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<std::vector<rclcpp::Parameter>>
  get_parameters(
    const std::vector<std::string> & names,
    std::function<
      void(std::shared_future<std::vector<rclcpp::Parameter>>)
    > callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<std::vector<rclcpp::ParameterType>>
  get_parameter_types(
    const std::vector<std::string> & names,
    std::function<
      void(std::shared_future<std::vector<rclcpp::ParameterType>>)
    > callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
  set_parameters(
    const std::vector<rclcpp::Parameter> & parameters,
    std::function<
      void(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>)
    > callback = nullptr);

  RCLCPP_PUBLIC
  std::shared_future<rcl_interfaces::msg::SetParametersResult>
  set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters,
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
    typename AllocatorT = std::allocator<void>>
  typename rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(
    CallbackT && callback,
    const rclcpp::QoS & qos = (
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events))
    ),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options = (
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
  ))
  {
    return this->on_parameter_event(
      this->node_topics_interface_,
      callback,
      qos,
      options);
  }

  /**
   * The NodeT type only needs to have a method called get_node_topics_interface()
   * which returns a shared_ptr to a NodeTopicsInterface, or be a
   * NodeTopicsInterface pointer itself.
   */
  template<
    typename CallbackT,
    typename NodeT,
    typename AllocatorT = std::allocator<void>>
  static typename rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(
    NodeT && node,
    CallbackT && callback,
    const rclcpp::QoS & qos = (
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_parameter_events))
    ),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options = (
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>()
  ))
  {
    return rclcpp::create_subscription<rcl_interfaces::msg::ParameterEvent>(
      node,
      "parameter_events",
      qos,
      std::forward<CallbackT>(callback),
      options);
  }

  /// Return if the parameter services are ready.
  /**
   * This method checks the following services:
   *  - get parameter
   *  - get parameter
   *  - set parameters
   *  - list parameters
   *  - describe parameters
   *
   * \return `true` if the service is ready, `false` otherwise
   */
  RCLCPP_PUBLIC
  bool
  service_is_ready() const;

  /// Wait for the services to be ready.
  /**
   * \param timeout maximum time to wait
   * \return `true` if the services are ready and the timeout is not over, `false` otherwise
   */
  template<typename RepT = int64_t, typename RatioT = std::milli>
  bool
  wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
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
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameterTypes>::SharedPtr
    get_parameter_types_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr
    set_parameters_atomically_client_;
  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr
    describe_parameters_client_;
  std::string remote_node_name_;
};

class SyncParametersClient
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SyncParametersClient)

  RCLCPP_PUBLIC
  explicit SyncParametersClient(
    rclcpp::Node::SharedPtr node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

  RCLCPP_PUBLIC
  SyncParametersClient(
    rclcpp::Executor::SharedPtr executor,
    rclcpp::Node::SharedPtr node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

  RCLCPP_PUBLIC
  explicit SyncParametersClient(
    rclcpp::Node * node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

  RCLCPP_PUBLIC
  SyncParametersClient(
    rclcpp::Executor::SharedPtr executor,
    rclcpp::Node * node,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

  RCLCPP_PUBLIC
  SyncParametersClient(
    rclcpp::Executor::SharedPtr executor,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
    const std::string & remote_node_name = "",
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_parameters);

  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter>
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
    if ((vars.size() != 1) || (vars[0].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)) {
      return parameter_not_found_handler();
    } else {
      return static_cast<T>(vars[0].get_value<T>());
    }
  }

  template<typename T>
  T
  get_parameter(const std::string & parameter_name, const T & default_value)
  {
    return get_parameter_impl(
      parameter_name,
      std::function<T()>([&default_value]() -> T {return default_value;}));
  }

  template<typename T>
  T
  get_parameter(const std::string & parameter_name)
  {
    return get_parameter_impl(
      parameter_name,
      std::function<T()>(
        [&parameter_name]() -> T
        {
          throw std::runtime_error("Parameter '" + parameter_name + "' is not set");
        })
    );
  }

  RCLCPP_PUBLIC
  std::vector<rclcpp::ParameterType>
  get_parameter_types(const std::vector<std::string> & parameter_names);

  RCLCPP_PUBLIC
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(const std::vector<rclcpp::Parameter> & parameters);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(const std::vector<rclcpp::Parameter> & parameters);

  RCLCPP_PUBLIC
  rcl_interfaces::msg::ListParametersResult
  list_parameters(
    const std::vector<std::string> & parameter_prefixes,
    uint64_t depth);

  template<typename CallbackT>
  typename rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(CallbackT && callback)
  {
    return async_parameters_client_->on_parameter_event(
      std::forward<CallbackT>(callback));
  }

  /**
   * The NodeT type only needs to have a method called get_node_topics_interface()
   * which returns a shared_ptr to a NodeTopicsInterface, or be a
   * NodeTopicsInterface pointer itself.
   */
  template<
    typename CallbackT,
    typename NodeT>
  static typename rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
  on_parameter_event(
    NodeT && node,
    CallbackT && callback)
  {
    return AsyncParametersClient::on_parameter_event(
      node,
      std::forward<CallbackT>(callback));
  }

  RCLCPP_PUBLIC
  bool
  service_is_ready() const
  {
    return async_parameters_client_->service_is_ready();
  }

  template<typename RepT = int64_t, typename RatioT = std::milli>
  bool
  wait_for_service(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return async_parameters_client_->wait_for_service(timeout);
  }

private:
  rclcpp::Executor::SharedPtr executor_;
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  AsyncParametersClient::SharedPtr async_parameters_client_;
};

}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_CLIENT_HPP_
