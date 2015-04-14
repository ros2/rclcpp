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
#include <thread>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/macros.hpp>
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
  get_name() const {return this->name_; }

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
  template<typename ServiceT>
  typename rclcpp::service::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    typename rclcpp::service::Service<ServiceT>::CallbackType callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  /* Create and return a Service. */
  template<typename ServiceT>
  typename rclcpp::service::Service<ServiceT>::SharedPtr
  create_service(
    const std::string & service_name,
    typename rclcpp::service::Service<ServiceT>::CallbackWithHeaderType callback_with_header,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr);

  template<typename ParamTypeT>
  std::shared_future<ParamTypeT>
  async_get_param(
    const std::string & node_name, const parameter::ParamName & key,
    std::function<void(std::shared_future<ParamTypeT>)> callback = nullptr);

  std::shared_future<std::vector<parameter::ParamContainer>>
  async_get_params(
    const std::string & node_name, const std::vector<parameter::ParamName> & parameter_names,
    std::function<void(std::shared_future<std::vector<parameter::ParamContainer>>)> callback =
    nullptr);

  std::shared_future<bool>
  async_has_param(const std::string & node_name, const parameter::ParamQuery & query,
    std::function<void(std::shared_future<bool>)> callback = nullptr);

  template<typename ParamTypeT>
  std::shared_future<bool>
  async_set_param(
    const std::string & node_name, const parameter::ParamName & key,
    const ParamTypeT & value, std::function<void(std::shared_future<bool>)> callback = nullptr);

  std::shared_future<bool>
  async_set_params(
    const std::string & node_name,
    const std::vector<parameter::ParamContainer> & key_values,
    std::function<void(std::shared_future<bool>)> callback = nullptr);

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

  void register_service(
    const std::string & service_name,
    std::shared_ptr<rclcpp::service::ServiceBase> serv_base_ptr,
    rclcpp::callback_group::CallbackGroup::SharedPtr group);

  std::map<parameter::ParamName, parameter::ParamContainer> params_;

  template<typename ParamTypeT>
  ParamTypeT get_param(const parameter::ParamName & key) const;

  std::vector<parameter::ParamContainer>
  get_params(const std::vector<parameter::ParamQuery> & query) const;

  bool
  has_param(const parameter::ParamQuery & query) const;

  template<typename ParamTypeT>
  void set_param(const parameter::ParamName & key, const ParamTypeT & value);
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
