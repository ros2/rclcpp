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

#ifndef RCLCPP_RCLCPP_NODE_IMPL_HPP_
#define RCLCPP_RCLCPP_NODE_IMPL_HPP_

#include <memory>
#include <string>

#include <ros_middleware_interface/functions.h>
#include <ros_middleware_interface/handles.h>

#include <ros_middleware_interface/get_type_support_handle.h>

#include <rclcpp/contexts/default_context.hpp>

#ifndef RCLCPP_RCLCPP_NODE_HPP_
#include "node.hpp"
#endif

using namespace rclcpp;
using namespace rclcpp::node;

using rclcpp::contexts::default_context::DefaultContext;

Node::Node(std::string node_name)
  : Node(node_name, DefaultContext::make_shared())
{}

Node::Node(std::string node_name, context::Context::SharedPtr context)
  : name_(node_name), context_(context), number_of_subscriptions_(0)
{
  node_handle_ = ::ros_middleware_interface::create_node();
  using rclcpp::callback_group::CallbackGroup;
  using rclcpp::callback_group::CallbackGroupType;
  default_callback_group_.reset(new CallbackGroup(
    "default_group", CallbackGroupType::NonThreadSafe));
  callback_groups_.push_back(default_callback_group_);
}

template<typename MessageT> publisher::Publisher::SharedPtr
Node::create_publisher(std::string topic_name, size_t queue_size)
{
  namespace rmi = ::ros_middleware_interface;

  auto type_support_handle = rmi::get_type_support_handle<MessageT>();
  auto publisher_handle = rmi::create_publisher(this->node_handle_,
                                                type_support_handle,
                                                topic_name.c_str());

  return publisher::Publisher::make_shared(publisher_handle);
}

template<typename MessageT>
typename subscription::Subscription<MessageT>::SharedPtr
Node::create_subscription(
  std::string topic_name,
  size_t queue_size,
  std::function<void(const std::shared_ptr<MessageT> &)> callback)
{
  namespace rmi = ::ros_middleware_interface;

  auto &type_support_handle = rmi::get_type_support_handle<MessageT>();
  auto subscriber_handle = rmi::create_subscriber(this->node_handle_,
                                                  type_support_handle,
                                                  topic_name.c_str());

  using namespace rclcpp::subscription;

  auto sub = Subscription<MessageT>::make_shared(subscriber_handle,
                                                 topic_name,
                                                 callback);
  auto sub_base_ptr = std::dynamic_pointer_cast<SubscriptionBase>(sub);
  this->default_callback_group_->add_subscription(sub_base_ptr);
  number_of_subscriptions_++;
  return sub;
}

#endif /* RCLCPP_RCLCPP_NODE_IMPL_HPP_ */
