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

#ifndef RCLCPP_RCLCPP_NODE_HPP_
#define RCLCPP_RCLCPP_NODE_HPP_

#include <list>
#include <memory>
#include <string>
#include <tuple>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>


#include <rcl_interfaces/SetParameters.h>

// Forward declaration of ROS middleware class
namespace rmw
{
struct rmw_node_t;
} // namespace rmw

namespace rclcpp
{

// Forward declaration for friend statement
namespace executor
{
class Executor;
} // namespace executor

namespace node
{

template<typename FunctionT>
struct function_traits
{
  static constexpr std::size_t arity =
    function_traits<decltype( & FunctionT::operator())>::arity - 1;

  template<std::size_t N>
  using argument_type =
      typename function_traits<decltype( & FunctionT::operator())>::template argument_type<N + 1>;
};

template<typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT(Args ...)>
{
  static constexpr std::size_t arity = sizeof ... (Args);

  template<std::size_t N>
  using argument_type = typename std::tuple_element<N, std::tuple<Args ...>>::type;
};

template<typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT (*)(Args ...)>: public function_traits<ReturnTypeT(Args ...)>
{};

template<typename ClassT, typename ReturnTypeT, typename ... Args>
struct function_traits<ReturnTypeT (ClassT::*)(Args ...) const>
  : public function_traits<ReturnTypeT(ClassT &, Args ...)>
{};

/* ROS Node Interface.
 *
 * This is the single point of entry for creating publishers and subscribers.
 */
class Node
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(Node);

  /* Create a node based on the node name. */
  Node(std::string node_name);
  /* Create a node based on the node name and a rclcpp::context::Context. */
  Node(std::string node_name, rclcpp::context::Context::SharedPtr context);

  /* Get the name of the node. */
  std::string
  get_name();

  /* Create and return a callback group. */
  rclcpp::callback_group::CallbackGroup::SharedPtr
  create_callback_group(rclcpp::callback_group::CallbackGroupType group_type);

  /* Create and return a Publisher. */
  template<typename MessageT>
  rclcpp::publisher::Publisher::SharedPtr
  create_publisher(std::string topic_name, size_t queue_size);

  /* Create and return a Subscription. */
  template<typename MessageT>
  typename rclcpp::subscription::Subscription<MessageT>::SharedPtr
  create_subscription(
    std::string topic_name,
    size_t queue_size,
    std::function<void(const std::shared_ptr<MessageT> &)> callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  /* Create a timer. */
  rclcpp::timer::WallTimer::SharedPtr
  create_wall_timer(
    std::chrono::nanoseconds period,
    rclcpp::timer::CallbackType callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  rclcpp::timer::WallTimer::SharedPtr
  create_wall_timer(
    std::chrono::duration<long double, std::nano> period,
    rclcpp::timer::CallbackType callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  typedef rclcpp::callback_group::CallbackGroup CallbackGroup;
  typedef std::weak_ptr<CallbackGroup> CallbackGroupWeakPtr;
  typedef std::list<CallbackGroupWeakPtr> CallbackGroupWeakPtrList;

  /* Create and return a Client. */
  template<typename ServiceT>
  typename rclcpp::client::Client<ServiceT>::SharedPtr
  create_client(
    const std::string & service_name,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  /* Create and return a Service. */
  template<typename ServiceT, typename FunctorT>
  typename rclcpp::service::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    FunctorT callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  const std::vector<rcl_interfaces::SetParametersResult> set_parameters(
    const std::vector<rcl_interfaces::Parameter> & parameters);

  const rcl_interfaces::SetParametersResult set_parameters_atomically(
    const std::vector<rcl_interfaces::Parameter> & parameters);

  const std::vector<rclcpp::parameter::ParameterVariant> get_parameters(
    const std::vector<std::string> & names);

  const std::vector<rcl_interfaces::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names);

  const std::vector<uint8_t> get_parameter_types(
    const std::vector<std::string> & names);

  const std::vector<rcl_interfaces::ListParametersResult> list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth);

private:
  RCLCPP_DISABLE_COPY(Node);

  bool
  group_in_node(callback_group::CallbackGroup::SharedPtr & group);

  std::string name_;

  rmw_node_t * node_handle_;

  rclcpp::context::Context::SharedPtr context_;

  CallbackGroup::SharedPtr default_callback_group_;
  CallbackGroupWeakPtrList callback_groups_;

  size_t number_of_subscriptions_;
  size_t number_of_timers_;
  size_t number_of_services_;
  size_t number_of_clients_;

  std::mutex mutex_;

  std::map<std::string, rclcpp::parameter::ParameterVariant> parameters_;

  template<
    typename ServiceT,
    typename FunctorT,
    typename std::enable_if<
      function_traits<FunctorT>::arity == 2
      >::type * = nullptr,
    typename std::enable_if<
      std::is_same<
        typename function_traits<FunctorT>::template argument_type<0>,
        typename std::shared_ptr<typename ServiceT::Request>
        >::value
      >::type * = nullptr,
    typename std::enable_if<
      std::is_same<
        typename function_traits<FunctorT>::template argument_type<1>,
        typename std::shared_ptr<typename ServiceT::Response>
        >::value
      >::type * = nullptr
    >
  typename rclcpp::service::Service<ServiceT>::SharedPtr
  create_service_internal(
    rmw_service_t * service_handle,
    const std::string & service_name,
    FunctorT callback)
  {
    typename rclcpp::service::Service<ServiceT>::CallbackType callback_without_header =
      callback;
    return service::Service<ServiceT>::make_shared(
      service_handle, service_name, callback_without_header);
  }

  template<
    typename ServiceT,
    typename FunctorT,
    typename std::enable_if<
      function_traits<FunctorT>::arity == 3
      >::type * = nullptr,
    typename std::enable_if<
      std::is_same<
        typename function_traits<FunctorT>::template argument_type<0>,
        std::shared_ptr<rmw_request_id_t>
        >::value
      >::type * = nullptr,
    typename std::enable_if<
      std::is_same<
        typename function_traits<FunctorT>::template argument_type<1>,
        typename std::shared_ptr<typename ServiceT::Request>
        >::value
      >::type * = nullptr,
    typename std::enable_if<
      std::is_same<
        typename function_traits<FunctorT>::template argument_type<2>,
        typename std::shared_ptr<typename ServiceT::Response>
        >::value
      >::type * = nullptr
    >
  typename rclcpp::service::Service<ServiceT>::SharedPtr
  create_service_internal(
    rmw_service_t * service_handle,
    const std::string & service_name,
    FunctorT callback)
  {
    typename rclcpp::service::Service<ServiceT>::CallbackWithHeaderType callback_with_header =
      callback;
    return service::Service<ServiceT>::make_shared(
      service_handle, service_name, callback_with_header);
  }
};

} /* namespace node */
} /* namespace rclcpp */

#define RCLCPP_REGISTER_NODE(Class) RMW_EXPORT \
  rclcpp::node::Node::SharedPtr \
  create_node() \
  { \
    return rclcpp::node::Node::SharedPtr(new Class( \
               rclcpp::contexts::default_context::DefaultContext:: \
               make_shared())); \
  }

#ifndef RCLCPP_RCLCPP_NODE_IMPL_HPP_
// Template implementations
#include "node_impl.hpp"
#endif

#endif /* RCLCPP_RCLCPP_NODE_HPP_ */
