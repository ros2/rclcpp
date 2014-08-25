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

#ifndef RCLCPP_RCLCPP_CALLBACK_GROUP_HPP_
#define RCLCPP_RCLCPP_CALLBACK_GROUP_HPP_

#include <string>
#include <vector>

#include <rclcpp/subscription.hpp>

namespace rclcpp
{

// Forward declarations for friend statement in class CallbackGroup
namespace node {class Node;}
namespace executor {class Executor;}

namespace callback_group
{

enum class CallbackGroupType {NonThreadSafe, ThreadSafe};

class CallbackGroup
{
  friend class rclcpp::node::Node;
  friend class rclcpp::executor::Executor;
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(CallbackGroup);

private:
  CallbackGroup(std::string group_name, CallbackGroupType group_type)
    : name_(group_name), type_(group_type)
  {}
  RCLCPP_DISABLE_COPY(CallbackGroup);

  void
  add_subscription(subscription::SubscriptionBase::SharedPtr &subscription_ptr)
  {
    subscription_ptrs_.push_back(subscription_ptr);
  }

  std::string name_;
  CallbackGroupType type_;
  std::vector<subscription::SubscriptionBase::SharedPtr> subscription_ptrs_;

};

} /* namespace callback_group */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_CALLBACK_GROUP_HPP_ */
