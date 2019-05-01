// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__SUBSCRIPTION_BASE_HPP_
#define RCLCPP__SUBSCRIPTION_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rcl/subscription.h"

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "rmw/rmw.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

namespace node_interfaces
{
class NodeTopicsInterface;
}  // namespace node_interfaces

namespace intra_process_manager
{
/**
 * IntraProcessManager is forward declared here, avoiding a circular inclusion between
 * `intra_process_manager.hpp` and `subscription_base.hpp`.
 */
class IntraProcessManager;
}

/// Virtual base class for subscriptions. This pattern allows us to iterate over different template
/// specializations of Subscription, among other things.
class SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(SubscriptionBase)

  /// Default constructor.
  /**
   * \param[in] node_handle The rcl representation of the node that owns this subscription.
   * \param[in] type_support_handle rosidl type support struct, for the Message type of the topic.
   * \param[in] topic_name Name of the topic to subscribe to.
   * \param[in] subscription_options options for the subscription.
   * \param[in] is_serialized is true if the message will be delivered still serialized
   */
  RCLCPP_PUBLIC
  SubscriptionBase(
    std::shared_ptr<rcl_node_t> node_handle,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rcl_subscription_options_t & subscription_options,
    bool is_serialized = false);

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~SubscriptionBase();

  /// Get the topic that this subscription is subscribed on.
  RCLCPP_PUBLIC
  const char *
  get_topic_name() const;

  RCLCPP_PUBLIC
  std::shared_ptr<rcl_subscription_t>
  get_subscription_handle();

  RCLCPP_PUBLIC
  const std::shared_ptr<rcl_subscription_t>
  get_subscription_handle() const;

  RCLCPP_PUBLIC
  virtual const std::shared_ptr<rcl_subscription_t>
  get_intra_process_subscription_handle() const;

  /// Get all the QoS event handlers associated with this subscription.
  /** \return The vector of QoS event handlers. */
  RCLCPP_PUBLIC
  const std::vector<std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
  get_event_handlers() const;

  /// Borrow a new message.
  /** \return Shared pointer to the fresh message. */
  virtual std::shared_ptr<void>
  create_message() = 0;

  /// Borrow a new serialized message
  /** \return Shared pointer to a rcl_message_serialized_t. */
  virtual std::shared_ptr<rcl_serialized_message_t>
  create_serialized_message() = 0;

  /// Check if we need to handle the message, and execute the callback if we do.
  /**
   * \param[in] message Shared pointer to the message to handle.
   * \param[in] message_info Metadata associated with this message.
   */
  virtual void
  handle_message(std::shared_ptr<void> & message, const rmw_message_info_t & message_info) = 0;

  /// Return the message borrowed in create_message.
  /** \param[in] message Shared pointer to the returned message. */
  virtual void
  return_message(std::shared_ptr<void> & message) = 0;

  /// Return the message borrowed in create_serialized_message.
  /** \param[in] message Shared pointer to the returned message. */
  virtual void
  return_serialized_message(std::shared_ptr<rcl_serialized_message_t> & message) = 0;

  virtual void
  handle_intra_process_message(
    rcl_interfaces::msg::IntraProcessMessage & ipm,
    const rmw_message_info_t & message_info) = 0;

  const rosidl_message_type_support_t &
  get_message_type_support_handle() const;

  bool
  is_serialized() const;

  /// Get matching publisher count.
  /** \return The number of publishers on this topic. */
  RCLCPP_PUBLIC
  size_t
  get_publisher_count() const;

  using IntraProcessManagerWeakPtr =
    std::weak_ptr<rclcpp::intra_process_manager::IntraProcessManager>;

  /// Implemenation detail.
  void setup_intra_process(
    uint64_t intra_process_subscription_id,
    IntraProcessManagerWeakPtr weak_ipm,
    const rcl_subscription_options_t & intra_process_options);

protected:
  template<typename EventCallbackT>
  void
  add_event_handler(
    const EventCallbackT & callback,
    const rcl_subscription_event_type_t event_type)
  {
    auto handler = std::make_shared<QOSEventHandler<EventCallbackT>>(
      callback,
      rcl_subscription_event_init,
      get_subscription_handle().get(),
      event_type);
    event_handlers_.emplace_back(handler);
  }

  std::shared_ptr<rcl_node_t> node_handle_;
  std::shared_ptr<rcl_subscription_t> subscription_handle_;
  std::shared_ptr<rcl_subscription_t> intra_process_subscription_handle_;

  std::vector<std::shared_ptr<rclcpp::QOSEventHandlerBase>> event_handlers_;

  bool use_intra_process_;
  IntraProcessManagerWeakPtr weak_ipm_;
  uint64_t intra_process_subscription_id_;

private:
  RCLCPP_DISABLE_COPY(SubscriptionBase)

  rosidl_message_type_support_t type_support_;
  bool is_serialized_;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_BASE_HPP_
