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

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

#include "rcl/event_callback.h"
#include "rcl/subscription.h"

#include "rmw/impl/cpp/demangle.hpp"
#include "rmw/rmw.h"

#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/detail/cpp_callback_trampoline.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/experimental/subscription_intra_process_base.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/network_flow_endpoint.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_content_filter_options.hpp"
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

  /// Constructor.
  /**
   * This accepts rcl_subscription_options_t instead of rclcpp::SubscriptionOptions because
   * rclcpp::SubscriptionOptions::to_rcl_subscription_options depends on the message type.
   *
   * \param[in] node_base NodeBaseInterface pointer used in parts of the setup.
   * \param[in] type_support_handle rosidl type support struct, for the Message type of the topic.
   * \param[in] topic_name Name of the topic to subscribe to.
   * \param[in] subscription_options Options for the subscription.
   * \param[in] is_serialized is true if the message will be delivered still serialized
   */
  RCLCPP_PUBLIC
  SubscriptionBase(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const rosidl_message_type_support_t & type_support_handle,
    const std::string & topic_name,
    const rcl_subscription_options_t & subscription_options,
    bool is_serialized = false);

  /// Destructor.
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
  std::shared_ptr<const rcl_subscription_t>
  get_subscription_handle() const;

  /// Get all the QoS event handlers associated with this subscription.
  /** \return The map of QoS event handlers. */
  RCLCPP_PUBLIC
  const
  std::unordered_map<rcl_subscription_event_type_t, std::shared_ptr<rclcpp::QOSEventHandlerBase>> &
  get_event_handlers() const;

  /// Get the actual QoS settings, after the defaults have been determined.
  /**
   * The actual configuration applied when using RMW_QOS_POLICY_*_SYSTEM_DEFAULT
   * can only be resolved after the creation of the subscription, and it
   * depends on the underlying rmw implementation.
   * If the underlying setting in use can't be represented in ROS terms,
   * it will be set to RMW_QOS_POLICY_*_UNKNOWN.
   * May throw runtime_error when an unexpected error occurs.
   *
   * \return The actual qos settings.
   * \throws std::runtime_error if failed to get qos settings
   */
  RCLCPP_PUBLIC
  rclcpp::QoS
  get_actual_qos() const;

  /// Take the next inter-process message from the subscription as a type erased pointer.
  /**
   * \sa Subscription::take() for details on how this function works.
   *
   * The only difference is that it takes a type erased pointer rather than a
   * reference to the exact message type.
   *
   * This type erased version facilitates using the subscriptions in a type
   * agnostic way using SubscriptionBase::create_message() and
   * SubscriptionBase::handle_message().
   *
   * \param[out] message_out The type erased message pointer into which take
   *   will copy the data.
   * \param[out] message_info_out The message info for the taken message.
   * \returns true if data was taken and is valid, otherwise false
   * \throws any rcl errors from rcl_take, \sa rclcpp::exceptions::throw_from_rcl_error()
   */
  RCLCPP_PUBLIC
  bool
  take_type_erased(void * message_out, rclcpp::MessageInfo & message_info_out);

  /// Take the next inter-process message, in its serialized form, from the subscription.
  /**
   * For now, if data is taken (written) into the message_out and
   * message_info_out then true will be returned.
   * Unlike Subscription::take(), taking data serialized is not possible via
   * intra-process for the time being, so it will not need to de-duplicate
   * data in any case.
   *
   * \param[out] message_out The serialized message data structure used to
   *   store the taken message.
   * \param[out] message_info_out The message info for the taken message.
   * \returns true if data was taken and is valid, otherwise false
   * \throws any rcl errors from rcl_take, \sa rclcpp::exceptions::throw_from_rcl_error()
   */
  RCLCPP_PUBLIC
  bool
  take_serialized(rclcpp::SerializedMessage & message_out, rclcpp::MessageInfo & message_info_out);

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
  std::shared_ptr<rclcpp::SerializedMessage>
  create_serialized_message() = 0;

  /// Check if we need to handle the message, and execute the callback if we do.
  /**
   * \param[in] message Shared pointer to the message to handle.
   * \param[in] message_info Metadata associated with this message.
   */
  RCLCPP_PUBLIC
  virtual
  void
  handle_message(std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info) = 0;

  RCLCPP_PUBLIC
  virtual
  void
  handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_message,
    const rclcpp::MessageInfo & message_info) = 0;

  RCLCPP_PUBLIC
  virtual
  void
  handle_loaned_message(void * loaned_message, const rclcpp::MessageInfo & message_info) = 0;

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
  return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) = 0;

  RCLCPP_PUBLIC
  const rosidl_message_type_support_t &
  get_message_type_support_handle() const;

  /// Return if the subscription is serialized
  /**
   * \return `true` if the subscription is serialized, `false` otherwise
   */
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

  /// Return the waitable for intra-process
  /**
   * \return the waitable sharedpointer for intra-process, or nullptr if intra-process is not setup.
   * \throws std::runtime_error if the intra process manager is destroyed
   */
  RCLCPP_PUBLIC
  rclcpp::Waitable::SharedPtr
  get_intra_process_waitable() const;

  /// Exchange state of whether or not a part of the subscription is used by a wait set.
  /**
   * Used to ensure parts of the subscription are not used with multiple wait
   * sets simultaneously.
   *
   * \param[in] pointer_to_subscription_part address of a subscription part
   * \param[in] in_use_state the new state to exchange, true means "now in use",
   *   and false means "no longer in use".
   * \returns the current "in use" state.
   * \throws std::invalid_argument If pointer_to_subscription_part is nullptr.
   * \throws std::runtime_error If the pointer given is not a pointer to one of
   *   the parts of the subscription which can be used with a wait set.
   */
  RCLCPP_PUBLIC
  bool
  exchange_in_use_by_wait_set_state(void * pointer_to_subscription_part, bool in_use_state);

  /// Get network flow endpoints
  /**
   * Describes network flow endpoints that this subscription is receiving messages on
   * \return vector of NetworkFlowEndpoint
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::NetworkFlowEndpoint>
  get_network_flow_endpoints() const;

  /// Set a callback to be called when each new message is received.
  /**
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.
   *
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * Calling it again will clear any previously set callback.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rmw_subscription_set_on_new_message_callback
   * \sa rcl_subscription_set_on_new_message_callback
   *
   * \param[in] callback functor to be called when a new message is received
   */
  void
  set_on_new_message_callback(std::function<void(size_t)> callback)
  {
    if (!callback) {
      throw std::invalid_argument(
              "The callback passed to set_on_new_message_callback "
              "is not callable.");
    }

    auto new_callback =
      [callback, this](size_t number_of_messages) {
        try {
          callback(number_of_messages);
        } catch (const std::exception & exception) {
          RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::SubscriptionBase@" << this <<
              " caught " << rmw::impl::cpp::demangle(exception) <<
              " exception in user-provided callback for the 'on new message' callback: " <<
              exception.what());
        } catch (...) {
          RCLCPP_ERROR_STREAM(
            node_logger_,
            "rclcpp::SubscriptionBase@" << this <<
              " caught unhandled exception in user-provided callback " <<
              "for the 'on new message' callback");
        }
      };

    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    // Set it temporarily to the new callback, while we replace the old one.
    // This two-step setting, prevents a gap where the old std::function has
    // been replaced but the middleware hasn't been told about the new one yet.
    set_on_new_message_callback(
      rclcpp::detail::cpp_callback_trampoline<const void *, size_t>,
      static_cast<const void *>(&new_callback));

    // Store the std::function to keep it in scope, also overwrites the existing one.
    on_new_message_callback_ = new_callback;

    // Set it again, now using the permanent storage.
    set_on_new_message_callback(
      rclcpp::detail::cpp_callback_trampoline<const void *, size_t>,
      static_cast<const void *>(&on_new_message_callback_));
  }

  /// Unset the callback registered for new messages, if any.
  void
  clear_on_new_message_callback()
  {
    std::lock_guard<std::recursive_mutex> lock(callback_mutex_);

    if (on_new_message_callback_) {
      set_on_new_message_callback(nullptr, nullptr);
      on_new_message_callback_ = nullptr;
    }
  }

  /// Set a callback to be called when each new intra-process message is received.
  /**
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.
   *
   * Calling it again will clear any previously set callback.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rclcpp::SubscriptionIntraProcessBase::set_on_ready_callback
   *
   * \param[in] callback functor to be called when a new message is received
   */
  void
  set_on_new_intra_process_message_callback(std::function<void(size_t)> callback)
  {
    if (!use_intra_process_) {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling set_on_new_intra_process_message_callback for subscription with IPC disabled");
      return;
    }

    if (!callback) {
      throw std::invalid_argument(
              "The callback passed to set_on_new_intra_process_message_callback "
              "is not callable.");
    }

    // The on_ready_callback signature has an extra `int` argument used to disambiguate between
    // possible different entities within a generic waitable.
    // We hide that detail to users of this method.
    std::function<void(size_t, int)> new_callback = std::bind(callback, std::placeholders::_1);
    subscription_intra_process_->set_on_ready_callback(new_callback);
  }

  /// Unset the callback registered for new intra-process messages, if any.
  void
  clear_on_new_intra_process_message_callback()
  {
    if (!use_intra_process_) {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling clear_on_new_intra_process_message_callback for subscription with IPC disabled");
      return;
    }

    subscription_intra_process_->clear_on_ready_callback();
  }

  /// Set a callback to be called when each new qos event instance occurs.
  /**
   * The callback receives a size_t which is the number of events that occurred
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if events occurred before any
   * callback was set.
   *
   * Since this callback is called from the middleware, you should aim to make
   * it fast and not blocking.
   * If you need to do a lot of work or wait for some other event, you should
   * spin it off to another thread, otherwise you risk blocking the middleware.
   *
   * Calling it again will clear any previously set callback.
   *
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the qos event
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \sa rclcpp::QOSEventHandlerBase::set_on_ready_callback
   *
   * \param[in] callback functor to be called when a new event occurs
   * \param[in] event_type identifier for the qos event we want to attach the callback to
   */
  void
  set_on_new_qos_event_callback(
    std::function<void(size_t)> callback,
    rcl_subscription_event_type_t event_type)
  {
    if (event_handlers_.count(event_type) == 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling set_on_new_qos_event_callback for non registered subscription event_type");
      return;
    }

    if (!callback) {
      throw std::invalid_argument(
              "The callback passed to set_on_new_qos_event_callback "
              "is not callable.");
    }

    // The on_ready_callback signature has an extra `int` argument used to disambiguate between
    // possible different entities within a generic waitable.
    // We hide that detail to users of this method.
    std::function<void(size_t, int)> new_callback = std::bind(callback, std::placeholders::_1);
    event_handlers_[event_type]->set_on_ready_callback(new_callback);
  }

  /// Unset the callback registered for new qos events, if any.
  void
  clear_on_new_qos_event_callback(rcl_subscription_event_type_t event_type)
  {
    if (event_handlers_.count(event_type) == 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("rclcpp"),
        "Calling clear_on_new_qos_event_callback for non registered event_type");
      return;
    }

    event_handlers_[event_type]->clear_on_ready_callback();
  }

  /// Check if content filtered topic feature of the subscription instance is enabled.
  /**
   * \return boolean flag indicating if the content filtered topic of this subscription is enabled.
   */
  RCLCPP_PUBLIC
  bool
  is_cft_enabled() const;

  /// Set the filter expression and expression parameters for the subscription.
  /**
   * \param[in] filter_expression A filter expression to set.
   *   \sa ContentFilterOptions::filter_expression
   *   An empty string ("") will clear the content filter setting of the subscription.
   * \param[in] expression_parameters Array of expression parameters to set.
   *   \sa ContentFilterOptions::expression_parameters
   * \throws RCLBadAlloc if memory cannot be allocated
   * \throws RCLError if an unexpect error occurs
   */
  RCLCPP_PUBLIC
  void
  set_content_filter(
    const std::string & filter_expression,
    const std::vector<std::string> & expression_parameters = {});

  /// Get the filter expression and expression parameters for the subscription.
  /**
   * \return rclcpp::ContentFilterOptions The content filter options to get.
   * \throws RCLBadAlloc if memory cannot be allocated
   * \throws RCLError if an unexpect error occurs
   */
  RCLCPP_PUBLIC
  rclcpp::ContentFilterOptions
  get_content_filter() const;

protected:
  template<typename EventCallbackT>
  void
  add_event_handler(
    const EventCallbackT & callback,
    const rcl_subscription_event_type_t event_type)
  {
    auto handler = std::make_shared<QOSEventHandler<EventCallbackT,
        std::shared_ptr<rcl_subscription_t>>>(
      callback,
      rcl_subscription_event_init,
      get_subscription_handle(),
      event_type);
    qos_events_in_use_by_wait_set_.insert(std::make_pair(handler.get(), false));
    event_handlers_.insert(std::make_pair(event_type, handler));
  }

  RCLCPP_PUBLIC
  void default_incompatible_qos_callback(QOSRequestedIncompatibleQoSInfo & info) const;

  RCLCPP_PUBLIC
  bool
  matches_any_intra_process_publishers(const rmw_gid_t * sender_gid) const;

  RCLCPP_PUBLIC
  void
  set_on_new_message_callback(rcl_event_callback_t callback, const void * user_data);

  rclcpp::node_interfaces::NodeBaseInterface * const node_base_;

  std::shared_ptr<rcl_node_t> node_handle_;
  std::shared_ptr<rcl_subscription_t> subscription_handle_;
  std::shared_ptr<rcl_subscription_t> intra_process_subscription_handle_;
  rclcpp::Logger node_logger_;

  std::unordered_map<rcl_subscription_event_type_t,
    std::shared_ptr<rclcpp::QOSEventHandlerBase>> event_handlers_;

  bool use_intra_process_;
  IntraProcessManagerWeakPtr weak_ipm_;
  uint64_t intra_process_subscription_id_;
  std::shared_ptr<rclcpp::experimental::SubscriptionIntraProcessBase> subscription_intra_process_;

private:
  RCLCPP_DISABLE_COPY(SubscriptionBase)

  rosidl_message_type_support_t type_support_;
  bool is_serialized_;

  std::atomic<bool> subscription_in_use_by_wait_set_{false};
  std::atomic<bool> intra_process_subscription_waitable_in_use_by_wait_set_{false};
  std::unordered_map<rclcpp::QOSEventHandlerBase *,
    std::atomic<bool>> qos_events_in_use_by_wait_set_;

  std::recursive_mutex callback_mutex_;
  std::function<void(size_t)> on_new_message_callback_{nullptr};
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_BASE_HPP_
