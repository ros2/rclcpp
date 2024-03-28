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

#include "rcl/subscription.h"

#include "rclcpp/exceptions.hpp"

namespace rclcpp
{

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
  any_callback_.dispatch(message, message_info);
}

void
GenericSubscription::handle_loaned_message(
  void * message, const rclcpp::MessageInfo & message_info)
{
  (void) message;
  (void) message_info;
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
  rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message)
{
  (void) message;
  throw rclcpp::exceptions::UnimplementedError(
          "return_dynamic_message is not implemented for GenericSubscription");
}

void
GenericSubscription::handle_dynamic_message(
  const rclcpp::dynamic_typesupport::DynamicMessage::SharedPtr & message,
  const rclcpp::MessageInfo & message_info)
{
  (void) message;
  (void) message_info;
  throw rclcpp::exceptions::UnimplementedError(
          "handle_dynamic_message is not implemented for GenericSubscription");
}

}  // namespace rclcpp
