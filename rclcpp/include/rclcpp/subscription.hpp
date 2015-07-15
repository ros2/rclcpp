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

#ifndef RCLCPP_RCLCPP_SUBSCRIPTION_HPP_
#define RCLCPP_RCLCPP_SUBSCRIPTION_HPP_

#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <rclcpp/macros.hpp>

namespace rclcpp
{

// Forward declaration is for friend statement in SubscriptionBase
namespace executor
{
class Executor;
} // namespace executor

namespace subscription
{

class SubscriptionBase
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(SubscriptionBase);

  SubscriptionBase(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_subscription_t * subscription_handle,
    std::string & topic_name,
    bool ignore_local_publications)
  : node_handle_(node_handle),
    subscription_handle_(subscription_handle),
    topic_name_(topic_name),
    ignore_local_publications_(ignore_local_publications)
  {
    // To avoid unused private member warnings.
    (void)ignore_local_publications_;
  }

  virtual ~SubscriptionBase()
  {
    if (subscription_handle_) {
      if (rmw_destroy_subscription(node_handle_.get(), subscription_handle_) != RMW_RET_OK) {
        std::stringstream ss;
        ss << "Error in destruction of rmw subscription handle: " <<
          rmw_get_error_string_safe() << '\n';
        (std::cerr << ss.str()).flush();
      }
    }
  }

  std::string get_topic_name()
  {
    return this->topic_name_;
  }

  virtual std::shared_ptr<void> create_message() = 0;
  virtual void handle_message(std::shared_ptr<void> & message) = 0;

private:
  RCLCPP_DISABLE_COPY(SubscriptionBase);

  std::shared_ptr<rmw_node_t> node_handle_;

  rmw_subscription_t * subscription_handle_;
  std::string topic_name_;
  bool ignore_local_publications_;

};

template<typename MessageT>
class Subscription : public SubscriptionBase
{
public:
  typedef std::function<void (const std::shared_ptr<MessageT> &)> CallbackType;
  RCLCPP_MAKE_SHARED_DEFINITIONS(Subscription);

  Subscription(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_subscription_t * subscription_handle,
    std::string & topic_name,
    bool ignore_local_publications,
    CallbackType callback)
  : SubscriptionBase(node_handle, subscription_handle, topic_name, ignore_local_publications),
    callback_(callback)
  {}

  std::shared_ptr<void> create_message()
  {
    return std::shared_ptr<void>(new MessageT());
  }

  void handle_message(std::shared_ptr<void> & message)
  {
    auto typed_message = std::static_pointer_cast<MessageT>(message);
    callback_(typed_message);
  }

private:
  RCLCPP_DISABLE_COPY(Subscription);

  CallbackType callback_;

};

} /* namespace subscription */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_SUBSCRIPTION_HPP_ */
