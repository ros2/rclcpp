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

#ifndef RCLCPP__ANY_EXECUTABLE_HPP_
#define RCLCPP__ANY_EXECUTABLE_HPP_

#include <memory>
#include <mutex>

#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace executor
{

struct AnyExecutable
{
  RCLCPP_SMART_PTR_DEFINITIONS(AnyExecutable);

  RCLCPP_PUBLIC
  AnyExecutable();

  RCLCPP_PUBLIC
  virtual ~AnyExecutable();

  bool is_one_field_set() const;

  rclcpp::subscription::SubscriptionBase::ConstSharedPtr get_subscription() const;
  rclcpp::subscription::SubscriptionBase::ConstSharedPtr get_subscription_intra_process() const;
  rclcpp::timer::TimerBase::ConstSharedPtr get_timer() const;
  rclcpp::service::ServiceBase::SharedPtr get_service() const;
  rclcpp::client::ClientBase::SharedPtr get_client() const;
  // These are used to keep the scope on the containing items
  rclcpp::callback_group::CallbackGroup::SharedPtr get_callback_group() const;
  rclcpp::node::Node::SharedPtr get_node() const;

  void set_subscription(const rclcpp::subscription::SubscriptionBase::ConstSharedPtr subscription);
  void set_subscription_intra_process(const rclcpp::subscription::SubscriptionBase::ConstSharedPtr subscription);
  void set_timer(const rclcpp::timer::TimerBase::ConstSharedPtr timer);
  void set_service(const rclcpp::service::ServiceBase::SharedPtr service);
  void set_client(const rclcpp::client::ClientBase::SharedPtr client);
  // These are used to keep the scope on the containing items
  void set_callback_group(const rclcpp::callback_group::CallbackGroup::SharedPtr callback_group);
  void set_node(const rclcpp::node::Node::SharedPtr node);

private:
  // Only one of the following pointers will be set.
  rclcpp::subscription::SubscriptionBase::ConstSharedPtr subscription_;
  rclcpp::subscription::SubscriptionBase::ConstSharedPtr subscription_intra_process_;
  rclcpp::timer::TimerBase::ConstSharedPtr timer_;
  rclcpp::service::ServiceBase::SharedPtr service_;
  rclcpp::client::ClientBase::SharedPtr client_;
  // These are used to keep the scope on the containing items
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_;
  rclcpp::node::Node::SharedPtr node_;
};

}  // namespace executor
}  // namespace rclcpp

#endif  // RCLCPP__ANY_EXECUTABLE_HPP_
