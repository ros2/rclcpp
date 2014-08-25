/* Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RCLCPP_RCLCPP_NODE_HPP_
#define RCLCPP_RCLCPP_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

namespace rclcpp
{

// Forward declaration for friend statement
namespace executor {class Executor;}

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
  get_name();

  /* Creates a named callback group. */
  void
  create_callback_group(
    std::string group_name,
    rclcpp::callback_group::CallbackGroupType callback_group_type);

  /* Return callback group if it exists, otherwise evaluates to false. */
  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_callback_group(std::string group_name);

  /* Create and return a Publisher. */
  template<typename MessageT> rclcpp::publisher::Publisher::SharedPtr
  create_publisher(std::string topic_name, size_t queue_size);

  /* Create and return a Subscription. */
  template<typename MessageT>
  typename rclcpp::subscription::Subscription<MessageT>::SharedPtr
  create_subscription(
    std::string topic_name,
    size_t queue_size,
    std::function<void(const std::shared_ptr<MessageT> &)> callback);

private:
  RCLCPP_DISABLE_COPY(Node);

  std::string name_;

  ros_middleware_interface::NodeHandle node_handle_;

  rclcpp::context::Context::SharedPtr context_;

  rclcpp::callback_group::CallbackGroup::SharedPtr default_callback_group_;
  std::vector<rclcpp::callback_group::CallbackGroup::SharedPtr> callback_groups_;

  size_t number_of_subscriptions_;

};

} /* namespace node */
} /* namespace rclcpp */

#ifndef RCLCPP_RCLCPP_NODE_IMPL_HPP_
// Template implementations
#include "node_impl.hpp"
#endif

#endif /* RCLCPP_RCLCPP_NODE_HPP_ */
