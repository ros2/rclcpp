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
  const std::string & serialization_library_name)
: serialization_support_(nullptr),
  dynamic_message_type_(nullptr),
  dynamic_message_(nullptr),
  description_(nullptr),
  rosidl_message_type_support_(nullptr)
{
  rosidl_message_type_support_t * ts = nullptr;
  rcl_ret_t ret;

  if (serialization_library_name.empty()) {
    ret = rcl_dynamic_message_type_support_handle_create(nullptr, &description, &ts);
  } else {
    ret = rcl_dynamic_message_type_support_handle_create(
      serialization_library_name.c_str(), &description, &ts);
  }
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("error initializing rosidl message type support");
  }
  if (!ts) {
    throw std::runtime_error("could not init rosidl message type support");
  }
  if (!ts->data) {
    throw std::runtime_error("could not init rosidl message type support impl");
  }
  if (ts->typesupport_identifier != rosidl_get_dynamic_typesupport_identifier()) {
    throw std::runtime_error("rosidl message type support is of the wrong type");
  }

  // NOTE(methylDragon): Not technically const correct, but since it's a const void *,
  //                     we do it anyway...
  auto ts_impl = static_cast<const rosidl_dynamic_message_type_support_impl_t *>(ts->data);

  // NOTE(methylDragon): We don't destroy the rosidl_message_type_support->data since its members
  //                     are managed by the passed in SharedPtr wrapper classes. We just delete it.
  rosidl_message_type_support_.reset(
    ts,
    [](rosidl_message_type_support_t * ts) -> void {
      auto ts_impl = static_cast<const rosidl_dynamic_message_type_support_impl_t *>(ts->data);
      auto allocator = rcl_get_default_allocator();

      // These are all C allocated
      allocator.deallocate(ts_impl->type_hash, &allocator.state);
      allocator.deallocate(
        const_cast<rosidl_dynamic_message_type_support_impl_t *>(ts_impl), &allocator.state);
      allocator.deallocate(ts, &allocator.state);
    }
  );

  manage_description_(ts_impl->type_description);
  serialization_support_ = DynamicSerializationSupport::make_shared(ts_impl->serialization_support);

  dynamic_message_type_ = DynamicMessageType::make_shared(
    get_shared_dynamic_serialization_support(), ts_impl->dynamic_message_type);

  dynamic_message_ = DynamicMessage::make_shared(
    get_shared_dynamic_serialization_support(), ts_impl->dynamic_message);

  if (!rosidl_message_type_support_) {
    throw std::runtime_error("could not init rosidl message type support.");
  }
}


DynamicMessageTypeSupport::DynamicMessageTypeSupport(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription & description)
: serialization_support_(serialization_support),
  dynamic_message_type_(nullptr),
  dynamic_message_(nullptr),
  description_(nullptr),
  rosidl_message_type_support_(nullptr)
{
  // Check null
  if (!serialization_support) {
    throw std::runtime_error("serialization_support cannot be nullptr.");
  }

  rosidl_message_type_support_t * ts = nullptr;

  auto type_hash = std::make_unique<rosidl_type_hash_t>();
  rcutils_ret_t hash_ret = rcl_calculate_type_hash(
    // TODO(methylDragon): Swap this out with the conversion function when it is ready
    reinterpret_cast<const type_description_interfaces__msg__TypeDescription *>(&description),
    type_hash.get());
  if (hash_ret != RCL_RET_OK || !type_hash) {
    throw std::runtime_error("failed to get type hash");
  }

  rcutils_ret_t ret = rosidl_dynamic_message_type_support_handle_create(
    serialization_support->get_rosidl_serialization_support(),
    type_hash.get(),  // type_hash
    &description,     // type_description
    nullptr,          // type_description_sources (not implemented for dynamic types)
    &ts);
  if (ret != RCUTILS_RET_OK || !ts) {
    throw std::runtime_error("could not init rosidl message type support");
  }
  if (!ts->data) {
    throw std::runtime_error("could not init rosidl message type support impl");
  }
  if (ts->typesupport_identifier != rosidl_get_dynamic_typesupport_identifier()) {
    throw std::runtime_error("rosidl message type support is of the wrong type");
  }

  auto ts_impl = static_cast<const rosidl_dynamic_message_type_support_impl_t *>(ts->data);

  // NOTE(methylDragon): We don't finalize the rosidl_message_type_support->data since its members
  //                     are managed by the passed in SharedPtr wrapper classes. We just delete it.
  rosidl_message_type_support_.reset(
    ts,
    [](rosidl_message_type_support_t * ts) -> void {
      auto ts_impl = static_cast<const rosidl_dynamic_message_type_support_impl_t *>(ts->data);

      // These are allocated with new
      delete ts_impl->type_hash;  // Only because we should've allocated it here
      delete ts_impl;
      delete ts;
    }
  );
  manage_description_(ts_impl->type_description);

  dynamic_message_type_ = DynamicMessageType::make_shared(
    get_shared_dynamic_serialization_support(), ts_impl->dynamic_message_type);

  dynamic_message_ = DynamicMessage::make_shared(
    get_shared_dynamic_serialization_support(), ts_impl->dynamic_message);

  if (!rosidl_message_type_support_) {
    throw std::runtime_error("could not init rosidl message type support.");
  }

  type_hash.release();
}


DynamicMessageTypeSupport::DynamicMessageTypeSupport(
  DynamicSerializationSupport::SharedPtr serialization_support,
  DynamicMessageType::SharedPtr dynamic_message_type,
  DynamicMessage::SharedPtr dynamic_message,
  const rosidl_runtime_c__type_description__TypeDescription & description)
: serialization_support_(serialization_support),
  dynamic_message_type_(dynamic_message_type),
  dynamic_message_(dynamic_message),
  description_(nullptr),
  rosidl_message_type_support_(nullptr)
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

  description_.reset(
    new rosidl_runtime_c__type_description__TypeDescription(),
    [](rosidl_runtime_c__type_description__TypeDescription * description) -> void {
      rosidl_runtime_c__type_description__TypeDescription__destroy(description);
    });
  if (!description_) {
    throw std::runtime_error("could not init type description.");
  }
  if (!rosidl_runtime_c__type_description__TypeDescription__copy(
      &description,
      description_.get()))
  {
    throw std::runtime_error("could not copy type description.");
  }

  // Check identifiers
  if (serialization_support->get_library_identifier() !=
    dynamic_message_type->get_library_identifier())
  {
    throw std::runtime_error(
            "serialization support library identifier does not match "
            "dynamic message type library identifier.");
  }
  if (dynamic_message_type->get_library_identifier() != dynamic_message->get_library_identifier()) {
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

  init_rosidl_message_type_support_(
    serialization_support_, dynamic_message_type_, dynamic_message_, description_.get());
  if (!rosidl_message_type_support_) {
    throw std::runtime_error("could not init rosidl message type support.");
  }
}


DynamicMessageTypeSupport::~DynamicMessageTypeSupport() {}


void
DynamicMessageTypeSupport::manage_description_(
  rosidl_runtime_c__type_description__TypeDescription * description)
{
  if (!description) {
    throw std::runtime_error("description cannot be nullptr");
  }
  description_.reset(
    description,
    [](rosidl_runtime_c__type_description__TypeDescription * description) -> void {
      rosidl_runtime_c__type_description__TypeDescription__destroy(description);
    });
}


// This looks like rmw_`dynamic_message_type_support_handle_create()`, but instead just aggregates
// already initialized objects
void
DynamicMessageTypeSupport::init_rosidl_message_type_support_(
  DynamicSerializationSupport::SharedPtr serialization_support,
  DynamicMessageType::SharedPtr dynamic_message_type,
  DynamicMessage::SharedPtr dynamic_message,
  rosidl_runtime_c__type_description__TypeDescription * description)
{
  bool middleware_supports_type_discovery = rmw_feature_supported(
    RMW_MIDDLEWARE_SUPPORTS_TYPE_DISCOVERY);

  if (!middleware_supports_type_discovery && !description) {
    throw std::runtime_error(
            "Middleware does not support type discovery! Deferred dynamic type"
            "message type support will never be populated! You must provide a type "
            "description.");
    return;
  }

  auto type_hash = std::make_unique<rosidl_type_hash_t>();
  rcutils_ret_t hash_ret = rcl_calculate_type_hash(
    // TODO(methylDragon): Swap this out with the conversion function when it is ready
    //   from https://github.com/ros2/rcl/pull/1052
    reinterpret_cast<type_description_interfaces__msg__TypeDescription *>(description),
    type_hash.get());
  if (hash_ret != RCL_RET_OK || !type_hash) {
    throw std::runtime_error("failed to get type hash");
  }

  rosidl_dynamic_message_type_support_impl_t * ts_impl =
    new rosidl_dynamic_message_type_support_impl_t {
    type_hash.get(),                                              // type_hash
    description,                                                  // type_description
    nullptr,    // NOTE(methylDragon): Not supported for now      // type_description_sources
    serialization_support->get_rosidl_serialization_support(),    // serialization_support
    dynamic_message_type->get_rosidl_dynamic_type(),              // dynamic_message_type
    dynamic_message->get_rosidl_dynamic_data()                    // dynamic_message
  };
  if (!ts_impl) {
    throw std::runtime_error(
            "Could not allocate rosidl_dynamic_message_type_support_impl_t struct");
    return;
  }

  // NOTE(methylDragon): We don't finalize the rosidl_message_type_support->data since its members
  //                     are managed by the passed in SharedPtr wrapper classes. We just delete it.
  rosidl_message_type_support_.reset(
    new rosidl_message_type_support_t{
    rosidl_get_dynamic_typesupport_identifier(),          // typesupport_identifier
    ts_impl,                                              // data
    get_message_typesupport_handle_function,              // func
    // get_type_hash_func
    rosidl_get_dynamic_message_type_support_type_hash_function,
    // get_type_description_func
    rosidl_get_dynamic_message_type_support_type_description_function,
    // get_type_description_sources_func
    rosidl_get_dynamic_message_type_support_type_description_sources_function
  },
    [](rosidl_message_type_support_t * ts) -> void {
      auto ts_impl = static_cast<const rosidl_dynamic_message_type_support_impl_t *>(ts->data);
      auto allocator = rcl_get_default_allocator();
      // Only because we should've allocated it here (also it's C allocated)
      allocator.deallocate(ts_impl->type_hash, &allocator.state);
      delete ts_impl;
    }
  );

  if (!rosidl_message_type_support_) {
    throw std::runtime_error("Could not allocate rosidl_message_type_support_t struct");
    delete ts_impl;
    return;
  }

  type_hash.release();
}


// GETTERS =========================================================================================
const std::string
DynamicMessageTypeSupport::get_library_identifier() const
{
  return serialization_support_->get_library_identifier();
}


rosidl_message_type_support_t *
DynamicMessageTypeSupport::get_rosidl_message_type_support()
{
  return rosidl_message_type_support_.get();
}


const rosidl_message_type_support_t *
DynamicMessageTypeSupport::get_rosidl_message_type_support() const
{
  return rosidl_message_type_support_.get();
}


std::shared_ptr<rosidl_message_type_support_t>
DynamicMessageTypeSupport::get_shared_rosidl_message_type_support()
{
  return rosidl_message_type_support_;
}


std::shared_ptr<const rosidl_message_type_support_t>
DynamicMessageTypeSupport::get_shared_rosidl_message_type_support() const
{
  return rosidl_message_type_support_;
}


rosidl_runtime_c__type_description__TypeDescription *
DynamicMessageTypeSupport::get_rosidl_runtime_c_type_description()
{
  return description_.get();
}


const rosidl_runtime_c__type_description__TypeDescription *
DynamicMessageTypeSupport::get_rosidl_runtime_c_type_description() const
{
  return description_.get();
}


std::shared_ptr<rosidl_runtime_c__type_description__TypeDescription>
DynamicMessageTypeSupport::get_shared_rosidl_runtime_c_type_description()
{
  return description_;
}


std::shared_ptr<const rosidl_runtime_c__type_description__TypeDescription>
DynamicMessageTypeSupport::get_shared_rosidl_runtime_c_type_description() const
{
  return description_;
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
