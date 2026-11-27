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

#include "rclcpp/generic_subscription.hpp"

#include <memory>
#include <string>
#include <type_traits>

#include "rcl/subscription.h"

#include "rclcpp/detail/resolve_intra_process_buffer_type.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/experimental/intra_process_manager.hpp"
#include "rclcpp/experimental/subscription_intra_process.hpp"
#include "rclcpp/logging.hpp"

#include "tracetools/tracetools.h"

namespace rclcpp
{

void
GenericSubscription::setup_generic_intra_process(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  rclcpp::IntraProcessBufferType buffer_type)
{
  // Check if the QoS is compatible with intra-process.
  // Unlike typed subscriptions, generic subscriptions (e.g. rosbag2 recorder) do not throw:
  // they keep working through the middleware, as they did before supporting intra-process.
  auto qos_profile = get_actual_qos();
  if (qos_profile.history() != rclcpp::HistoryPolicy::KeepLast || qos_profile.depth() == 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "intra-process communication disabled for generic subscription on topic '%s': "
      "it requires keep last history qos policy and non-zero depth", get_topic_name());
    return;
  }

  using SubscriptionIntraProcessT = rclcpp::experimental::SubscriptionIntraProcess<
    rclcpp::SerializedMessage, rclcpp::SerializedMessage>;
  static_assert(
    std::is_base_of<
      rclcpp::experimental::IntraProcessManager::GenericSubscriptionIntraProcessBuffer,
      SubscriptionIntraProcessT>::value,
    "the intra-process manager expects a different buffer type for generic subscriptions");

  // First create a SubscriptionIntraProcess which will be given to the intra-process manager.
  auto context = node_base->get_context();
  subscription_intra_process_ = std::make_shared<SubscriptionIntraProcessT>(
    any_callback_,
    std::make_shared<std::allocator<void>>(),
    context,
    this->get_topic_name(),  // important to get like this, as it has the fully-qualified name
    qos_profile,
    rclcpp::detail::resolve_intra_process_buffer_type(buffer_type, any_callback_));
  TRACETOOLS_TRACEPOINT(
    rclcpp_subscription_init,
    static_cast<const void *>(get_subscription_handle().get()),
    static_cast<const void *>(subscription_intra_process_.get()));

  // Add it to the intra process manager.
  using rclcpp::experimental::IntraProcessManager;
  auto ipm = context->get_sub_context<IntraProcessManager>();
  uint64_t intra_process_subscription_id =
    ipm->add_generic_subscription(subscription_intra_process_);
  this->setup_intra_process(intra_process_subscription_id, ipm);
}

std::shared_ptr<void>
GenericSubscription::create_message()
{
  return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage>
GenericSubscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>(0);
}

void
GenericSubscription::disable_callbacks()
{
  SubscriptionBase::disable_callbacks();
  any_callback_.disable();
  if (subscription_intra_process_) {
    subscription_intra_process_->disable_callbacks();
  }
  for (const auto & [_, event_ptr] : event_handlers_) {
    if (event_ptr) {
      event_ptr->disable();
    }
  }
}

void
GenericSubscription::enable_callbacks()
{
  SubscriptionBase::enable_callbacks();
  any_callback_.enable();
  if (subscription_intra_process_) {
    subscription_intra_process_->enable_callbacks();
  }
  for (const auto & [_, event_ptr] : event_handlers_) {
    if (event_ptr) {
      event_ptr->enable();
    }
  }
}

void
GenericSubscription::handle_message(
  std::shared_ptr<void> &,
  const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_message is not implemented for GenericSubscription");
}

void
GenericSubscription::handle_serialized_message(
  const std::shared_ptr<rclcpp::SerializedMessage> & message,
  const rclcpp::MessageInfo & message_info)
{
  if (matches_any_intra_process_publishers(&message_info.get_rmw_message_info().publisher_gid)) {
    // In this case, the message will be delivered via intra process and
    // we should ignore this copy of the message.
    return;
  }

  any_callback_.dispatch(message, message_info);
}

void
GenericSubscription::handle_loaned_message(
  [[maybe_unused]] void * message,
  [[maybe_unused]] const rclcpp::MessageInfo & message_info)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_loaned_message is not implemented for GenericSubscription");
}

void
GenericSubscription::return_message(std::shared_ptr<void> & message)
{
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
  return_serialized_message(typed_message);
}

void
GenericSubscription::return_serialized_message(
  std::shared_ptr<rclcpp::SerializedMessage> & message)
{
  message.reset();
}


// DYNAMIC TYPE ====================================================================================
// TODO(methylDragon): Reorder later
rclcpp::dynamic_typesupport::DynamicMessageType::SharedPtr
GenericSubscription::get_shared_dynamic_message_type()
{
  throw rclcpp::exceptions::UnimplementedError(
          "get_shared_dynamic_message_type is not implemented for GenericSubscription");
}

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
GenericSubscription::get_shared_dynamic_message()
{
  throw rclcpp::exceptions::UnimplementedError(
          "get_shared_dynamic_message is not implemented for GenericSubscription");
}

rclcpp::dynamic_typesupport::DynamicSerializationSupport::SharedPtr
GenericSubscription::get_shared_dynamic_serialization_support()
{
  throw rclcpp::exceptions::UnimplementedError(
          "get_shared_dynamic_serialization_support is not implemented for GenericSubscription");
}

rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr
GenericSubscription::create_dynamic_message()
{
  throw rclcpp::exceptions::UnimplementedError(
          "create_dynamic_message is not implemented for GenericSubscription");
}

void
GenericSubscription::return_dynamic_message(
  [[maybe_unused]] rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message)
{
  throw rclcpp::exceptions::UnimplementedError(
          "return_dynamic_message is not implemented for GenericSubscription");
}

void
GenericSubscription::handle_dynamic_message(
  [[maybe_unused]] const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message,
  [[maybe_unused]] const rclcpp::MessageInfo & message_info)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_dynamic_message is not implemented for GenericSubscription");
}

}  // namespace rclcpp
