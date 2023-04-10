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

#include <rcl/allocator.h>
#include <rcutils/logging_macros.h>
#include <rmw/dynamic_message_type_support.h>
#include <rmw/ret_types.h>
#include <rosidl_dynamic_typesupport/api/serialization_support.h>

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/exceptions.hpp"

using rclcpp::dynamic_typesupport::DynamicSerializationSupport;

// CONSTRUCTION ====================================================================================
DynamicSerializationSupport::DynamicSerializationSupport(rcl_allocator_t allocator)
: DynamicSerializationSupport::DynamicSerializationSupport("", allocator) {}

DynamicSerializationSupport::DynamicSerializationSupport(
  const std::string & serialization_library_name,
  rcl_allocator_t allocator)
: rosidl_serialization_support_(
    rosidl_dynamic_typesupport_get_zero_initialized_serialization_support())
{
  rmw_ret_t ret = RMW_RET_ERROR;

  if (serialization_library_name.empty()) {
    ret = rmw_init_serialization_support(NULL, &allocator, &rosidl_serialization_support_);
  } else {
    ret = rmw_init_serialization_support(
      serialization_library_name.c_str(), &allocator, &rosidl_serialization_support_);
  }
  if (ret != RCL_RET_OK) {
    std::string error_msg =
      std::string("could not initialize new serialization support object: ") +
      rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(error_msg);
  }
}

DynamicSerializationSupport::DynamicSerializationSupport(
  rosidl_dynamic_typesupport_serialization_support_t && rosidl_serialization_support)
: rosidl_serialization_support_(std::move(rosidl_serialization_support)) {}

DynamicSerializationSupport::DynamicSerializationSupport(
  DynamicSerializationSupport && other) noexcept
: rosidl_serialization_support_(std::exchange(
      other.rosidl_serialization_support_,
      rosidl_dynamic_typesupport_get_zero_initialized_serialization_support())) {}

DynamicSerializationSupport &
DynamicSerializationSupport::operator=(DynamicSerializationSupport && other) noexcept
{
  std::swap(rosidl_serialization_support_, other.rosidl_serialization_support_);
  return *this;
}

DynamicSerializationSupport::~DynamicSerializationSupport()
{
  rosidl_dynamic_typesupport_serialization_support_fini(&rosidl_serialization_support_);
}


// GETTERS =========================================================================================
const std::string
DynamicSerializationSupport::get_serialization_library_identifier() const
{
  return std::string(
    rosidl_dynamic_typesupport_serialization_support_get_library_identifier(
      &rosidl_serialization_support_));
}

rosidl_dynamic_typesupport_serialization_support_t &
DynamicSerializationSupport::get_rosidl_serialization_support()
{
  return rosidl_serialization_support_;
}

const rosidl_dynamic_typesupport_serialization_support_t &
DynamicSerializationSupport::get_rosidl_serialization_support() const
{
  return rosidl_serialization_support_;
}
