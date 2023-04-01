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

#include <rosidl_dynamic_typesupport/api/serialization_support.h>

#include <memory>
#include <string>

#include "rcutils/logging_macros.h"
#include "rmw/dynamic_message_typesupport.h"
#include "rmw/ret_types.h"

#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/exceptions.hpp"


using rclcpp::dynamic_typesupport::DynamicSerializationSupport;

// CONSTRUCTION ====================================================================================
DynamicSerializationSupport::DynamicSerializationSupport(
  const std::string & serialization_library_name)
: rosidl_serialization_support_(nullptr)
{
  rosidl_dynamic_typesupport_serialization_support_t * rosidl_serialization_support = nullptr;
  rmw_ret_t ret = RMW_RET_ERROR;

  if (serialization_library_name.empty()) {
    ret = rmw_get_serialization_support(NULL, &rosidl_serialization_support);
  } else {
    ret = rmw_get_serialization_support(
      serialization_library_name.c_str(), &rosidl_serialization_support);
  }

  if (ret != RMW_RET_OK || !rosidl_serialization_support) {
    throw std::runtime_error("could not create new serialization support object");
  }

  rosidl_serialization_support_.reset(
    rosidl_serialization_support,
    // Custom deleter
    [](rosidl_dynamic_typesupport_serialization_support_t * rosidl_serialization_support) -> void {
      rosidl_dynamic_typesupport_serialization_support_destroy(rosidl_serialization_support);
    });
}


DynamicSerializationSupport::DynamicSerializationSupport(
  rosidl_dynamic_typesupport_serialization_support_t * rosidl_serialization_support)
: rosidl_serialization_support_(nullptr)
{
  if (!rosidl_serialization_support) {
    throw std::runtime_error("serialization support cannot be nullptr!");
  }

  // Custom deleter
  rosidl_serialization_support_.reset(
    rosidl_serialization_support,
    [](rosidl_dynamic_typesupport_serialization_support_t * rosidl_serialization_support) -> void {
      rosidl_dynamic_typesupport_serialization_support_destroy(rosidl_serialization_support);
    });
}


DynamicSerializationSupport::DynamicSerializationSupport(
  std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t> rosidl_serialization_support)
: rosidl_serialization_support_(rosidl_serialization_support) {}


DynamicSerializationSupport::DynamicSerializationSupport(
  DynamicSerializationSupport && other) noexcept
: rosidl_serialization_support_(std::exchange(other.rosidl_serialization_support_, nullptr)) {}


DynamicSerializationSupport &
DynamicSerializationSupport::operator=(DynamicSerializationSupport && other) noexcept
{
  std::swap(rosidl_serialization_support_, other.rosidl_serialization_support_);
  return *this;
}


DynamicSerializationSupport::~DynamicSerializationSupport() {}


// GETTERS =========================================================================================
const std::string
DynamicSerializationSupport::get_library_identifier() const
{
  return std::string(
    rosidl_dynamic_typesupport_serialization_support_get_library_identifier(
      rosidl_serialization_support_.get()));
}


rosidl_dynamic_typesupport_serialization_support_t *
DynamicSerializationSupport::get_rosidl_serialization_support()
{
  return rosidl_serialization_support_.get();
}


const rosidl_dynamic_typesupport_serialization_support_t *
DynamicSerializationSupport::get_rosidl_serialization_support() const
{
  return rosidl_serialization_support_.get();
}


std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t>
DynamicSerializationSupport::get_shared_rosidl_serialization_support()
{
  return std::shared_ptr<rosidl_dynamic_typesupport_serialization_support_t>(
    shared_from_this(), rosidl_serialization_support_.get());
}


std::shared_ptr<const rosidl_dynamic_typesupport_serialization_support_t>
DynamicSerializationSupport::get_shared_rosidl_serialization_support() const
{
  return std::shared_ptr<const rosidl_dynamic_typesupport_serialization_support_t>(
    shared_from_this(), rosidl_serialization_support_.get());
}
