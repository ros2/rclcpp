// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/dynamic_subscription.hpp"

#include <memory>
#include <string>

#include "rcl/subscription.h"

#include "rclcpp/exceptions.hpp"

namespace rclcpp
{

std::shared_ptr<void> DynamicSubscription::create_message()
{
  return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage> DynamicSubscription::create_serialized_message()
{
  return std::make_shared<rclcpp::SerializedMessage>(0);
}

void DynamicSubscription::handle_message(
  std::shared_ptr<void> &, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_message is not implemented for DynamicSubscription");
}

void DynamicSubscription::handle_serialized_message(
  const std::shared_ptr<rclcpp::SerializedMessage> &, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_serialized_message is not implemented for DynamicSubscription");
}

void DynamicSubscription::handle_loaned_message(
  void *, const rclcpp::MessageInfo &)
{
  throw rclcpp::exceptions::UnimplementedError(
          "handle_loaned_message is not implemented for DynamicSubscription");
}

void DynamicSubscription::return_message(std::shared_ptr<void> & message)
{
  auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
  return_serialized_message(typed_message);
}

void DynamicSubscription::return_serialized_message(
  std::shared_ptr<rclcpp::SerializedMessage> & message)
{
  message.reset();
}


// RUNTIME TYPE ====================================================================================
// TODO(methylDragon): Re-order later
std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_t>
DynamicSubscription::get_dynamic_type()
{
  auto ts_impl = (rmw_dynamic_typesupport_impl_t *)(ts_.data);

  // no-op deleter because the lifetime is managed by the typesupport outside
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_t>(
    ts_impl->dynamic_type, [](rosidl_dynamic_typesupport_dynamic_type_t *){}
  );
};

// NOTE(methylDragon): Should we store a separate copy of dynamic data in the sub so it isn't tied
//                     to the typesupport instead?
//                     If that's the case, will there ever be a lifetime contention between a sub
//                     that manages the data and the callback/user usage of the data? 
std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>
DynamicSubscription::get_dynamic_data()
{
  auto ts_impl = (rmw_dynamic_typesupport_impl_t *)(ts_.data);

  // no-op deleter because the lifetime is managed by the typesupport outside
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>(
    ts_impl->dynamic_data, [](rosidl_dynamic_typesupport_dynamic_data_t *){}
  );
};

std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t> DynamicSubscription::get_serialization_support()
{
  auto ts_impl = (rmw_dynamic_typesupport_impl_t *)(ts_.data);

  // no-op deleter because the lifetime is managed by the typesupport outside
  return std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t>(
    ts_impl->serialization_support, [](rosidl_dynamic_typesupport_serialization_support_t *){}
  );
};

void DynamicSubscription::handle_dynamic_message(
  const std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t> & serialization_support,
  const std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t> & dyn_data,
  const rclcpp::MessageInfo &)
{
  callback_(serialization_support, dyn_data);
}

}  // namespace rclcpp
