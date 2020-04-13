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

#include "rmw/rmw.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

namespace node_interfaces
{
class NodeBaseInterface;
}  // namespace node_interfaces

namespace experimental
{
/**
 * IntraProcessManager is forward declared here, avoiding a circular inclusion between
 * `intra_process_manager.hpp` and `subscription_base.hpp`.
 */
class IntraProcessManager;
}  // namespace experimental

/// Virtual base class for subscriptions. This pattern allows us to iterate over different template
/// specializations of Subscription, among other things.
class SubscriptionBase : public std::enable_shared_from_this<SubscriptionBase>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(SubscriptionBase)

  /// Default constructor.
  /**
   * \param[in] node_base NodeBaseInterface pointer used in parts of the setup.
   * \param[in] type_support_handle rosidl type support struct, for the Message type of the topic.
   * \param[in] topic_name Name of the topic to subscribe to.
   * \param[in] subscription_options options for the subscription.
   * \param[in] is_serialized is true if the message will be delivered still serialized
   */
  RCLCPP_PUBLIC
  SubscriptionBase(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
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

  /// Get all the QoS event handlers associated with this subscription.
  /** \return The vector of QoS event handlers. */
  RCLCPP_PUBLIC
  const std::vector<std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
  get_event_handlers() const;

  /// Get the actual QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the publisher, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual qos settings.
   */
  RCLCPP_PUBLIC
  rclcpp::QoS
  get_actual_qos() const;

  /// Borrow a new message.
  /** \return Shared pointer to the fresh message. */
  RCLCPP_PUBLIC
  virtual
  std::shared_ptr<void>
  create_message() = 0;

  /// Borrow a new serialized message
  /** \return Shared pointer to a rcl_message_serialized_t. */
  RCLCPP_PUBLIC
  virtual
  std::shared_ptr<rcl_serialized_message_t>
  create_serialized_message() = 0;

  /// Check if we need to handle the message, and execute the callback if we do.
  /**
   * \param[in] message Shared pointer to the message to handle.
   * \param[in] message_info Metadata associated with this message.
   */
  RCLCPP_PUBLIC
  virtual
  void
  handle_message(std::shared_ptr<void> & message, const rmw_message_info_t & message_info) = 0;

  RCLCPP_PUBLIC
  virtual
  void
  handle_loaned_message(void * loaned_message, const rmw_message_info_t & message_info) = 0;

  /// Return the message borrowed in create_message.
  /** \param[in] message Shared pointer to the returned message. */
  RCLCPP_PUBLIC
  virtual
  void
  return_message(std::shared_ptr<void> & message) = 0;

  /// Return the message borrowed in create_serialized_message.
  /** \param[in] message Shared pointer to the returned message. */
  RCLCPP_PUBLIC
  virtual
  void
  return_serialized_message(std::shared_ptr<rcl_serialized_message_t> & message) = 0;

  RCLCPP_PUBLIC
  const rosidl_message_type_support_t &
  get_message_type_support_handle() const;

  RCLCPP_PUBLIC
  bool
  is_serialized() const;

  /// Get matching publisher count.
  /** \return The number of publishers on this topic. */
  RCLCPP_PUBLIC
  size_t
  get_publisher_count() const;

  /// Check if subscription instance can loan messages.
  /**
   * Depending on the middleware and the message type, this will return true if the middleware
   * can allocate a ROS message instance.
   *
   * \return boolean flag indicating if middleware can loan messages.
   */
  RCLCPP_PUBLIC
  bool
  can_loan_messages() const;

  using IntraProcessManagerWeakPtr =
    std::weak_ptr<rclcpp::experimental::IntraProcessManager>;

  /// Implemenation detail.
  RCLCPP_PUBLIC
  void
  setup_intra_process(
    uint64_t intra_process_subscription_id,
    IntraProcessManagerWeakPtr weak_ipm);

  /// Return the waitable for intra-process, or nullptr if intra-process is not setup.
  RCLCPP_PUBLIC
  rclcpp::Waitable::SharedPtr
  get_intra_process_waitable() const;

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

  RCLCPP_PUBLIC
  void default_incompatible_qos_callback(QOSRequestedIncompatibleQoSInfo & info) const;

  RCLCPP_PUBLIC
  bool
  matches_any_intra_process_publishers(const rmw_gid_t * sender_gid) const;

  rclcpp::node_interfaces::NodeBaseInterface * const node_base_;

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
