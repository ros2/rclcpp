// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2021, Apex.AI Inc.
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

#ifndef RCLCPP__GENERIC_SUBSCRIPTION_HPP_
#define RCLCPP__GENERIC_SUBSCRIPTION_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rcpputils/shared_library.hpp"

#include "rclcpp/callback_group.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// %Subscription for serialized messages whose type is not known at compile time.
/**
 * Since the type is not known at compile time, this is not a template, and the dynamic library
 * containing type support information has to be identified and loaded based on the type name.
 *
 * It does not support intra-process handling.
 */
class GenericSubscription : public rclcpp::SubscriptionBase
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(GenericSubscription)

  /// Constructor.
  /**
   * In order to properly subscribe to a topic, this subscription needs to be added to
   * the node_topic_interface of the node passed into this constructor.
   *
   * \sa rclcpp::Node::create_generic_subscription() or rclcpp::create_generic_subscription() for
   * creating an instance of this class and adding it to the node_topic_interface.
   *
   * \param node_base Pointer to parent node's NodeBaseInterface
   * \param ts_lib Type support library, needs to correspond to topic_type
   * \param topic_name Topic name
   * \param topic_type Topic type
   * \param qos %QoS settings
   * \param callback Callback for new messages of serialized form
   * \param options %Subscription options.
   * Not all subscription options are currently respected, the only relevant options for this
   * subscription are `event_callbacks`, `use_default_callbacks`, `ignore_local_publications`, and
   * `%callback_group`.
   */
  template<typename AllocatorT = std::allocator<void>>
  GenericSubscription(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::shared_ptr<rcpputils::SharedLibrary> ts_lib,
    const std::string & topic_name,
    const std::string & topic_type,
    const rclcpp::QoS & qos,
    AnySubscriptionCallback<rclcpp::SerializedMessage, AllocatorT> callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options)
  : SubscriptionBase(
      node_base,
      *rclcpp::get_message_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *ts_lib),
      topic_name,
      options.to_rcl_subscription_options(qos),
      options.event_callbacks,
      options.use_default_callbacks,
      DeliveredMessageKind::SERIALIZED_MESSAGE),
    callback_([callback](
        std::shared_ptr<const rclcpp::SerializedMessage> serialized_message,
        const rclcpp::MessageInfo & message_info) mutable {
        callback.dispatch(serialized_message, message_info);
      }),
    ts_lib_(ts_lib)
  {
    // Setup intra process publishing if requested.
    if (rclcpp::detail::resolve_use_intra_process(options, *node_base)) {
      using rclcpp::detail::resolve_intra_process_buffer_type;
      using MessageT = rclcpp::SerializedMessage;
      using SubscribedType = rclcpp::SerializedMessage;
      using ROSMessageT = rclcpp::SerializedMessage;

      using SubscribedTypeAllocatorTraits = allocator::AllocRebind<SubscribedType, AllocatorT>;
      using SubscribedTypeAllocator = typename SubscribedTypeAllocatorTraits::allocator_type;
      using SubscribedTypeDeleter = allocator::Deleter<SubscribedTypeAllocator, SubscribedType>;

      // Check if the QoS is compatible with intra-process.
      auto qos_profile = get_actual_qos();
      if (qos_profile.history() != rclcpp::HistoryPolicy::KeepLast) {
        throw std::invalid_argument(
                "intraprocess communication allowed only with keep last history qos policy");
      }
      if (qos_profile.depth() == 0) {
        throw std::invalid_argument(
                "intraprocess communication is not allowed with 0 depth qos policy");
      }
      if (qos_profile.durability() != rclcpp::DurabilityPolicy::Volatile) {
        throw std::invalid_argument(
                "intraprocess communication allowed only with volatile durability");
      }

      using SubscriptionIntraProcessT = rclcpp::experimental::SubscriptionIntraProcess<
        MessageT,
        SubscribedType,
        SubscribedTypeAllocator,
        SubscribedTypeDeleter,
        ROSMessageT,
        AllocatorT>;

      // First create a SubscriptionIntraProcess which will be given to the intra-process manager.
      auto context = node_base->get_context();
      subscription_intra_process_ = std::make_shared<SubscriptionIntraProcessT>(
        callback,
        options.get_allocator(),
        context,
        this->get_topic_name(),  // important to get like this, as it has the fully-qualified name
        qos_profile,
        resolve_intra_process_buffer_type(options.intra_process_buffer_type, callback));
      TRACEPOINT(
        rclcpp_subscription_init,
        static_cast<const void *>(get_subscription_handle().get()),
        static_cast<const void *>(subscription_intra_process_.get()));

      // Add it to the intra process manager.
      using rclcpp::experimental::IntraProcessManager;
      auto ipm = context->get_sub_context<IntraProcessManager>();
      uint64_t intra_process_subscription_id = ipm->add_subscription(subscription_intra_process_);
      this->setup_intra_process(intra_process_subscription_id, ipm);
    }
  }

  RCLCPP_PUBLIC
  virtual ~GenericSubscription() = default;

  // Same as create_serialized_message() as the subscription is to serialized_messages only
  RCLCPP_PUBLIC
  std::shared_ptr<void> create_message() override;

  RCLCPP_PUBLIC
  std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override;

  /// Cast the message to a rclcpp::SerializedMessage and call the callback.
  RCLCPP_PUBLIC
  void handle_message(
    std::shared_ptr<void> & message, const rclcpp::MessageInfo & message_info) override;

  /// Handle dispatching rclcpp::SerializedMessage to user callback.
  RCLCPP_PUBLIC
  void
  handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_message,
    const rclcpp::MessageInfo & message_info) override;

  /// This function is currently not implemented.
  RCLCPP_PUBLIC
  void handle_loaned_message(
    void * loaned_message, const rclcpp::MessageInfo & message_info) override;

  // Same as return_serialized_message() as the subscription is to serialized_messages only
  RCLCPP_PUBLIC
  void return_message(std::shared_ptr<void> & message) override;

  RCLCPP_PUBLIC
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> & message) override;


  // DYNAMIC TYPE ==================================================================================
  RCLCPP_PUBLIC
  rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr get_shared_dynamic_message_type()
  override;

  RCLCPP_PUBLIC
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr get_shared_dynamic_message() override;

  RCLCPP_PUBLIC
  rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
  get_shared_dynamic_serialization_support() override;

  RCLCPP_PUBLIC
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr create_dynamic_message() override;

  RCLCPP_PUBLIC
  void return_dynamic_message(
    rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message) override;

  RCLCPP_PUBLIC
  void handle_dynamic_message(
    const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message,
    const rclcpp::MessageInfo & message_info) override;

private:
  RCLCPP_DISABLE_COPY(GenericSubscription)

  std::function<void(
      std::shared_ptr<const rclcpp::SerializedMessage>,
      const rclcpp::MessageInfo)> callback_;
  // The type support library should stay loaded, so it is stored in the GenericSubscription
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
};

}  // namespace rclcpp

#endif  // RCLCPP__GENERIC_SUBSCRIPTION_HPP_
