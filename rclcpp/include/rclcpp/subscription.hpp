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

#include <rcl_interfaces/msg/intra_process_message.hpp>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <rclcpp/macros.hpp>
#include <rclcpp/message_memory_strategy.hpp>

namespace rclcpp
{

// Forward declaration is for friend statement in SubscriptionBase
namespace executor
{
class Executor;
} // namespace executor

namespace node
{
class Node;
} // namespace node

namespace subscription
{

class SubscriptionBase
{
  friend class rclcpp::executor::Executor;

public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(SubscriptionBase);

  SubscriptionBase(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_subscription_t * subscription_handle,
    const std::string & topic_name,
    bool ignore_local_publications)
  : intra_process_subscription_handle_(nullptr),
    node_handle_(node_handle),
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
    if (intra_process_subscription_handle_) {
      auto ret = rmw_destroy_subscription(node_handle_.get(), intra_process_subscription_handle_);
      if (ret != RMW_RET_OK) {
        std::stringstream ss;
        ss << "Error in destruction of rmw intra process subscription handle: " <<
          rmw_get_error_string_safe() << '\n';
        (std::cerr << ss.str()).flush();
      }
    }
  }

  const std::string & get_topic_name() const
  {
    return this->topic_name_;
  }

  virtual std::shared_ptr<void> create_message() = 0;
  virtual void handle_message(std::shared_ptr<void> & message, const rmw_gid_t * sender_id) = 0;
  virtual void return_message(std::shared_ptr<void> & message) = 0;
  virtual void handle_intra_process_message(rcl_interfaces::msg::IntraProcessMessage & ipm) = 0;

protected:
  rmw_subscription_t * intra_process_subscription_handle_;

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
  friend class rclcpp::node::Node;

public:
  using CallbackType = std::function<void(const std::shared_ptr<MessageT> &)>;
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription);

  Subscription(
    std::shared_ptr<rmw_node_t> node_handle,
    rmw_subscription_t * subscription_handle,
    const std::string & topic_name,
    bool ignore_local_publications,
    CallbackType callback,
    typename message_memory_strategy::MessageMemoryStrategy<MessageT>::SharedPtr memory_strategy =
    message_memory_strategy::MessageMemoryStrategy<MessageT>::create_default())
  : SubscriptionBase(node_handle, subscription_handle, topic_name, ignore_local_publications),
    callback_(callback),
    message_memory_strategy_(memory_strategy),
    get_intra_process_message_callback_(nullptr),
    matches_any_intra_process_publishers_(nullptr)
  {}

  void set_message_memory_strategy(
    typename message_memory_strategy::MessageMemoryStrategy<MessageT>::SharedPtr message_memory_strategy)
  {
    message_memory_strategy_ = message_memory_strategy;
  }

  std::shared_ptr<void> create_message()
  {
    return message_memory_strategy_->borrow_message();
  }

  void handle_message(std::shared_ptr<void> & message, const rmw_gid_t * sender_id)
  {
    if (matches_any_intra_process_publishers_) {
      if (matches_any_intra_process_publishers_(sender_id)) {
        // In this case, the message will be delivered via intra process and
        // we should ignore this copy of the message.
        return;
      }
    }
    auto typed_message = std::static_pointer_cast<MessageT>(message);
    callback_(typed_message);
  }

  void return_message(std::shared_ptr<void> & message)
  {
    auto typed_message = std::static_pointer_cast<MessageT>(message);
    message_memory_strategy_->return_message(typed_message);
  }

  void handle_intra_process_message(rcl_interfaces::msg::IntraProcessMessage & ipm)
  {
    if (!get_intra_process_message_callback_) {
      // throw std::runtime_error(
      //   "handle_intra_process_message called before setup_intra_process");
      // TODO(wjwwood): for now, this could mean that intra process was just not enabled.
      // However, this can only really happen if this node has it disabled, but the other doesn't.
      return;
    }
    std::unique_ptr<MessageT> msg;
    get_intra_process_message_callback_(
      ipm.publisher_id,
      ipm.message_sequence,
      intra_process_subscription_id_,
      msg);
    if (!msg) {
      // This either occurred because the publisher no longer exists or the
      // message requested is no longer being stored.
      // TODO(wjwwood): should we notify someone of this? log error, log warning?
      return;
    }
    typename MessageT::SharedPtr shared_msg = std::move(msg);
    callback_(shared_msg);
  }

private:
  typedef
    std::function<
      void (uint64_t, uint64_t, uint64_t, std::unique_ptr<MessageT> &)
    > GetMessageCallbackType;
  typedef std::function<bool (const rmw_gid_t *)> MatchesAnyPublishersCallbackType;

  void setup_intra_process(
    uint64_t intra_process_subscription_id,
    rmw_subscription_t * intra_process_subscription,
    GetMessageCallbackType get_message_callback,
    MatchesAnyPublishersCallbackType matches_any_publisher_callback)
  {
    intra_process_subscription_id_ = intra_process_subscription_id;
    intra_process_subscription_handle_ = intra_process_subscription;
    get_intra_process_message_callback_ = get_message_callback;
    matches_any_intra_process_publishers_ = matches_any_publisher_callback;
  }

  RCLCPP_DISABLE_COPY(Subscription);

  CallbackType callback_;
  typename message_memory_strategy::MessageMemoryStrategy<MessageT>::SharedPtr
  message_memory_strategy_;

  GetMessageCallbackType get_intra_process_message_callback_;
  MatchesAnyPublishersCallbackType matches_any_intra_process_publishers_;
  uint64_t intra_process_subscription_id_;

};

} /* namespace subscription */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_SUBSCRIPTION_HPP_ */
