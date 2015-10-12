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

#include <rcl_interfaces/msg/list_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rosidl_generator_cpp/message_type_support.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/function_traits.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>


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
/// Node is the single point of entry for creating publishers and subscribers.
class Node
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(Node);

  /// Create a new node with the specified name.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   */
  Node(const std::string & node_name, bool use_intra_process_comms = false);

  /// Create a node based on the node name and a rclcpp::context::Context.
  /**
   * \param[in] node_name Name of the node.
   * \param[in] context The context for the node (usually represents the state of a process).
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   */
  Node(
    const std::string & node_name, rclcpp::context::Context::SharedPtr context,
    bool use_intra_process_comms = false);

  /// Get the name of the node.
  // \return The name of the node.
  const std::string &
  get_name() const {return name_; }

  /// Create and return a callback group.
  rclcpp::callback_group::CallbackGroup::SharedPtr
  create_callback_group(rclcpp::callback_group::CallbackGroupType group_type);

  /// Create and return a Publisher.
  /**
   * \param[in] topic_name The topic for this publisher to publish on.
   * \param[in] qos_profile The quality of service profile to pass on to the rmw implementation.
   * \return Shared pointer to the created publisher.
   */
  template<typename MessageT>
  rclcpp::publisher::Publisher::SharedPtr
  create_publisher(
    const std::string & topic_name, const rmw_qos_profile_t & qos_profile);

  /// Create and return a Subscription.
  /**
   * \param[in] topic_name The topic to subscribe on.
   * \param[in] qos_profile The quality of service profile to pass on to the rmw implementation.
   * \param[in] callback The user-defined callback function.
   * \param[in] group The callback group for this subscription. NULL for no callback group.
   * \param[in] ignore_local_publications True to ignore local publications.
   * \param[in] msg_mem_strat The message memory strategy to use for allocating messages.
   * \return Shared pointer to the created subscription.
   */
  /* TODO(jacquelinekay):
     Windows build breaks when static member function passed as default
     argument to msg_mem_strat, nullptr is a workaround.
   */
  template<typename MessageT, typename CallbackT>
  typename rclcpp::subscription::Subscription<MessageT>::SharedPtr
  create_subscription(
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile,
    CallbackT callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT>::SharedPtr
    msg_mem_strat = nullptr);

  template<typename MessageT>
  typename rclcpp::subscription::Subscription<MessageT>::SharedPtr
  create_subscription_with_unique_ptr_callback(
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile,
    typename rclcpp::subscription::AnySubscriptionCallback<MessageT>::UniquePtrCallback callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<MessageT>::SharedPtr
    msg_mem_strat = nullptr);

  /// Create a timer.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  rclcpp::timer::WallTimer::SharedPtr
  create_wall_timer(
    std::chrono::nanoseconds period,
    rclcpp::timer::CallbackType callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  /// Create a timer with a sub-nanosecond precision update period.
  /**
   * \param[in] period Time interval between triggers of the callback.
   * \param[in] callback User-defined callback function.
   * \param[in] group Callback group to execute this timer's callback in.
   */
  // TODO(wjwwood): reenable this once I figure out why the demo doesn't build with it.
  // rclcpp::timer::WallTimer::SharedPtr
  // create_wall_timer(
  //   std::chrono::duration<long double, std::nano> period,
  //   rclcpp::timer::CallbackType callback,
  //   rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  using CallbackGroup = rclcpp::callback_group::CallbackGroup;
  using CallbackGroupWeakPtr = std::weak_ptr<CallbackGroup>;
  using CallbackGroupWeakPtrList = std::list<CallbackGroupWeakPtr>;

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

  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters(
    const std::vector<rclcpp::parameter::ParameterVariant> & parameters);

  rcl_interfaces::msg::SetParametersResult set_parameters_atomically(
    const std::vector<rclcpp::parameter::ParameterVariant> & parameters);

  std::vector<rclcpp::parameter::ParameterVariant> get_parameters(
    const std::vector<std::string> & names) const;

  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names) const;

  std::vector<uint8_t> get_parameter_types(
    const std::vector<std::string> & names) const;

  rcl_interfaces::msg::ListParametersResult list_parameters(
    const std::vector<std::string> & prefixes, uint64_t depth) const;

  std::map<std::string, std::string> get_topic_names_and_types() const;

  size_t count_publishers(const std::string & topic_name) const;

  size_t count_subscribers(const std::string & topic_name) const;

private:
  RCLCPP_DISABLE_COPY(Node);

  static const rosidl_message_type_support_t * ipm_ts_;

  bool
  group_in_node(callback_group::CallbackGroup::SharedPtr group);

  std::string name_;

  std::shared_ptr<rmw_node_t> node_handle_;

  rclcpp::context::Context::SharedPtr context_;

  CallbackGroup::SharedPtr default_callback_group_;
  CallbackGroupWeakPtrList callback_groups_;

  size_t number_of_subscriptions_;
  size_t number_of_timers_;
  size_t number_of_services_;
  size_t number_of_clients_;

  bool use_intra_process_comms_;

  mutable std::mutex mutex_;

  std::map<std::string, rclcpp::parameter::ParameterVariant> parameters_;

  publisher::Publisher::SharedPtr events_publisher_;

  template<typename MessageT>
  typename subscription::Subscription<MessageT>::SharedPtr
  create_subscription_internal(
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile,
    rclcpp::subscription::AnySubscriptionCallback<MessageT> callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group,
    bool ignore_local_publications,
    typename message_memory_strategy::MessageMemoryStrategy<MessageT>::SharedPtr msg_mem_strat);

  template<
    typename ServiceT,
    typename FunctorT,
    std::size_t Arity = 2
  >
  typename std::enable_if<
    rclcpp::arity_comparator<Arity, FunctorT>::value,
    typename rclcpp::service::Service<ServiceT>::SharedPtr
  >::type
  create_service_internal(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_service_t * service_handle,
    const std::string & service_name,
    FunctorT callback,
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
    >::type * = nullptr)
  {
    typename rclcpp::service::Service<ServiceT>::CallbackType callback_without_header =
      callback;
    return service::Service<ServiceT>::make_shared(
      node_handle, service_handle, service_name, callback_without_header);
  }

  template<
    typename ServiceT,
    typename FunctorT,
    std::size_t Arity = 3
  >
  typename std::enable_if<
    arity_comparator<Arity, FunctorT>::value,
    typename rclcpp::service::Service<ServiceT>::SharedPtr
  >::type
  create_service_internal(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_service_t * service_handle,
    const std::string & service_name,
    FunctorT callback,
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
    >::type * = nullptr)
  {
    typename rclcpp::service::Service<ServiceT>::CallbackWithHeaderType callback_with_header =
      callback;
    return service::Service<ServiceT>::make_shared(
      node_handle, service_handle, service_name, callback_with_header);
  }
};

const rosidl_message_type_support_t * Node::ipm_ts_ =
  rosidl_generator_cpp::get_message_type_support_handle<rcl_interfaces::msg::IntraProcessMessage>();

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
