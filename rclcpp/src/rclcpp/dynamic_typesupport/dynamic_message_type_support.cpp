// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <rosidl_dynamic_typesupport/identifier.h>
#include <rosidl_dynamic_typesupport/types.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/type_description_utils.h>
#include <rosidl_runtime_c/type_description/type_description__functions.h>
#include <rosidl_runtime_c/type_description/type_description__struct.h>
#include <rosidl_runtime_c/type_description/type_source__functions.h>
#include <rosidl_runtime_c/type_description/type_source__struct.h>

#include <memory>
#include <string>

#include "rcl/allocator.h"
#include "rcl/dynamic_message_type_support.h"
#include "rcl/type_hash.h"
#include "rcl/types.h"
#include "rcutils/logging_macros.h"
#include "rcutils/types/rcutils_ret.h"
#include "rmw/dynamic_message_type_support.h"

#include "rclcpp/dynamic_typesupport/dynamic_message.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_message_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_message_type_support.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

using rclcpp::dynamic_typesupport::DynamicMessage;
using rclcpp::dynamic_typesupport::DynamicMessageType;
using rclcpp::dynamic_typesupport::DynamicMessageTypeSupport;
using rclcpp::dynamic_typesupport::DynamicSerializationSupport;

// CONSTRUCTION ====================================================================================
DynamicMessageTypeSupport::DynamicMessageTypeSupport(
  const rosidl_runtime_c__type_description__TypeDescription & description,
  const std::string & serialization_library_name,
  rcl_allocator_t allocator)
: serialization_support_(nullptr),
  dynamic_message_type_(nullptr),
  dynamic_message_(nullptr),
  rosidl_message_type_support_(rosidl_get_zero_initialized_message_type_support_handle())
{
  rcl_ret_t ret;
  if (serialization_library_name.empty()) {
    ret = rcl_dynamic_message_type_support_handle_init(
      nullptr, &description, &allocator, &rosidl_message_type_support_);
  } else {
    ret = rcl_dynamic_message_type_support_handle_init(
      serialization_library_name.c_str(), &description, &allocator, &rosidl_message_type_support_);
  }
  if (ret != RCL_RET_OK) {
    std::string error_msg =
      std::string("error initializing rosidl message type support:\n") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_msg);
  }
  if (rosidl_message_type_support_.typesupport_identifier !=
    rosidl_get_dynamic_typesupport_identifier())
  {
    throw std::runtime_error("rosidl message type support is of the wrong type");
  }

  auto ts_impl = static_cast<const rosidl_dynamic_message_type_support_impl_t *>(
    rosidl_message_type_support_.data);

  serialization_support_ = DynamicSerializationSupport::make_shared(ts_impl->serialization_support);

  dynamic_message_type_ = DynamicMessageType::make_shared(
    get_shared_dynamic_serialization_support(), ts_impl->dynamic_message_type);

  dynamic_message_ = DynamicMessage::make_shared(
    get_shared_dynamic_serialization_support(), ts_impl->dynamic_message);
}

DynamicMessageTypeSupport::DynamicMessageTypeSupport(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription & description,
  rcl_allocator_t allocator)
: serialization_support_(serialization_support),
  dynamic_message_type_(nullptr),
  dynamic_message_(nullptr),
  rosidl_message_type_support_(rosidl_get_zero_initialized_message_type_support_handle())
{
  // Check null
  if (!serialization_support) {
    throw std::runtime_error("serialization_support cannot be nullptr.");
  }

  rosidl_type_hash_t type_hash;
  rcutils_ret_t hash_ret = rcl_calculate_type_hash(
    // TODO(methylDragon): Swap this out with the conversion function when it is ready
    reinterpret_cast<const type_description_interfaces__msg__TypeDescription *>(&description),
    &type_hash);
  if (hash_ret != RCL_RET_OK) {
    throw std::runtime_error("failed to get type hash");
  }

  rcutils_ret_t ret = rosidl_dynamic_message_type_support_handle_init(
    serialization_support->get_rosidl_serialization_support(),
    &type_hash,       // type_hash
    &description,     // type_description
    nullptr,          // type_description_sources (not implemented for dynamic types)
    &allocator,
    &rosidl_message_type_support_);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not init rosidl message type support");
  }
  if (rosidl_message_type_support_.typesupport_identifier !=
    rosidl_get_dynamic_typesupport_identifier())
  {
    throw std::runtime_error("rosidl message type support is of the wrong type");
  }

  auto ts_impl = static_cast<const rosidl_dynamic_message_type_support_impl_t *>(
    rosidl_message_type_support_.data);

  dynamic_message_type_ = DynamicMessageType::make_shared(
    get_shared_dynamic_serialization_support(), ts_impl->dynamic_message_type);

  dynamic_message_ = DynamicMessage::make_shared(
    get_shared_dynamic_serialization_support(), ts_impl->dynamic_message);
}

DynamicMessageTypeSupport::DynamicMessageTypeSupport(
  DynamicSerializationSupport::SharedPtr serialization_support,
  DynamicMessageType::SharedPtr dynamic_message_type,
  DynamicMessage::SharedPtr dynamic_message,
  const rosidl_runtime_c__type_description__TypeDescription & description,
  rcl_allocator_t allocator)
: serialization_support_(serialization_support),
  dynamic_message_type_(dynamic_message_type),
  dynamic_message_(dynamic_message),
  rosidl_message_type_support_(rosidl_get_zero_initialized_message_type_support_handle())
{
  // Check null
  if (!serialization_support) {
    throw std::runtime_error("serialization_support cannot be nullptr.");
  }
  if (!dynamic_message_type) {
    throw std::runtime_error("dynamic_message_type cannot be nullptr.");
  }
  if (!dynamic_message) {
    throw std::runtime_error("dynamic_message cannot be nullptr.");
  }

  // Check identifiers
  if (serialization_support->get_serialization_library_identifier() !=
    dynamic_message_type->get_serialization_library_identifier())
  {
    throw std::runtime_error(
            "serialization support library identifier does not match "
            "dynamic message type library identifier.");
  }
  if (dynamic_message_type->get_serialization_library_identifier() !=
    dynamic_message->get_serialization_library_identifier())
  {
    throw std::runtime_error(
            "dynamic message type library identifier does not match "
            "dynamic message library identifier.");
  }

  // Check pointers
  /* *INDENT-OFF* */
  if (serialization_support->get_rosidl_serialization_support() !=
      dynamic_message_type
        ->get_shared_dynamic_serialization_support()
        ->get_rosidl_serialization_support())
  {
    throw std::runtime_error("serialization support pointer does not match dynamic message type's");
  }
  if (dynamic_message_type
        ->get_shared_dynamic_serialization_support()
        ->get_rosidl_serialization_support() !=
      dynamic_message_type
        ->get_shared_dynamic_serialization_support()
        ->get_rosidl_serialization_support())
  {
    throw std::runtime_error("serialization support does not match pointer dynamic message type's");
  }
  /* *INDENT-ON* */

  rosidl_type_hash_t type_hash;
  rcutils_ret_t hash_ret = rcl_calculate_type_hash(
    // TODO(methylDragon): Swap this out with the conversion function when it is ready
    //   from https://github.com/ros2/rcl/pull/1052
    reinterpret_cast<const type_description_interfaces__msg__TypeDescription *>(&description),
    &type_hash);
  if (hash_ret != RCL_RET_OK) {
    std::string error_msg = std::string("failed to get type hash:\n") + rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_msg);
  }

  auto ts_impl = static_cast<rosidl_dynamic_message_type_support_impl_t *>(
    allocator.zero_allocate(1, sizeof(rosidl_dynamic_message_type_support_impl_t), &allocator.state)
  );
  if (!ts_impl) {
    throw std::runtime_error("could not allocate rosidl_message_type_support_t");
  }

  ts_impl->allocator = allocator;
  ts_impl->type_hash = type_hash;
  if (!rosidl_runtime_c__type_description__TypeDescription__copy(
      &description, &ts_impl->type_description))
  {
    throw std::runtime_error("could not copy type description");
  }
  // ts_impl->type_description_sources  = // Not used

  ts_impl->serialization_support = serialization_support->get_rosidl_serialization_support();
  ts_impl->dynamic_message_type = dynamic_message_type->get_rosidl_dynamic_type();
  ts_impl->dynamic_message = dynamic_message->get_rosidl_dynamic_data();

  rosidl_message_type_support_ = {
    rosidl_get_dynamic_typesupport_identifier(),            // typesupport_identifier
    ts_impl,                                                // data
    get_message_typesupport_handle_function,                // func
    // get_type_hash_func
    rosidl_get_dynamic_message_type_support_type_hash_function,
    // get_type_description_func
    rosidl_get_dynamic_message_type_support_type_description_function,
    // get_type_description_sources_func
    rosidl_get_dynamic_message_type_support_type_description_sources_function
  };
}

DynamicMessageTypeSupport::~DynamicMessageTypeSupport()
{
  if (!rosidl_message_type_support_.data) {
    return;
  }

  // We only partially finalize the rosidl_message_type_support->data since its pointer members are
  // managed by their respective SharedPtr wrapper classes.
  auto ts_impl = static_cast<rosidl_dynamic_message_type_support_impl_t *>(
    const_cast<void *>(rosidl_message_type_support_.data)
  );
  rcutils_allocator_t allocator = ts_impl->allocator;

  rosidl_runtime_c__type_description__TypeDescription__fini(&ts_impl->type_description);
  rosidl_runtime_c__type_description__TypeSource__Sequence__fini(
    &ts_impl->type_description_sources);
  allocator.deallocate(static_cast<void *>(ts_impl), &allocator.state);  // Always C allocated
}


// GETTERS =========================================================================================
const std::string
DynamicMessageTypeSupport::get_serialization_library_identifier() const
{
  return serialization_support_->get_serialization_library_identifier();
}

rosidl_message_type_support_t &
DynamicMessageTypeSupport::get_rosidl_message_type_support()
{
  return rosidl_message_type_support_;
}

const rosidl_message_type_support_t &
DynamicMessageTypeSupport::get_const_rosidl_message_type_support()
{
  return rosidl_message_type_support_;
}

const rosidl_message_type_support_t &
DynamicMessageTypeSupport::get_const_rosidl_message_type_support() const
{
  return rosidl_message_type_support_;
}

const rosidl_runtime_c__type_description__TypeDescription &
DynamicMessageTypeSupport::get_rosidl_runtime_c_type_description() const
{
  auto ts_impl = static_cast<const rosidl_dynamic_message_type_support_impl_t *>(
    get_const_rosidl_message_type_support().data
  );
  return ts_impl->type_description;
}

DynamicSerializationSupport::SharedPtr
DynamicMessageTypeSupport::get_shared_dynamic_serialization_support()
{
  return serialization_support_;
}

DynamicSerializationSupport::ConstSharedPtr
DynamicMessageTypeSupport::get_shared_dynamic_serialization_support() const
{
  return serialization_support_;
}

DynamicMessageType::SharedPtr
DynamicMessageTypeSupport::get_shared_dynamic_message_type()
{
  return dynamic_message_type_;
}

DynamicMessageType::ConstSharedPtr
DynamicMessageTypeSupport::get_shared_dynamic_message_type() const
{
  return dynamic_message_type_;
}

DynamicMessage::SharedPtr
DynamicMessageTypeSupport::get_shared_dynamic_message()
{
  return dynamic_message_;
}

DynamicMessage::ConstSharedPtr
DynamicMessageTypeSupport::get_shared_dynamic_message() const
{
  return dynamic_message_;
}
