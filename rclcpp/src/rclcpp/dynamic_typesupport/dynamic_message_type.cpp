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

#include <rosidl_dynamic_typesupport/api/dynamic_data.h>
#include <rosidl_dynamic_typesupport/api/dynamic_type.h>
#include <rosidl_dynamic_typesupport/api/serialization_support.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <memory>
#include <string>

#include "rcutils/logging_macros.h"

#include "rclcpp/dynamic_typesupport/dynamic_message.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_message_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_message_type_builder.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/exceptions.hpp"


using rclcpp::dynamic_typesupport::DynamicMessage;
using rclcpp::dynamic_typesupport::DynamicMessageType;
using rclcpp::dynamic_typesupport::DynamicMessageTypeBuilder;
using rclcpp::dynamic_typesupport::DynamicSerializationSupport;


// CONSTRUCTION ====================================================================================
DynamicMessageType::DynamicMessageType(DynamicMessageTypeBuilder::SharedPtr dynamic_type_builder)
: DynamicMessageType::DynamicMessageType(
    dynamic_type_builder, dynamic_type_builder->get_rosidl_dynamic_type_builder().allocator) {}


DynamicMessageType::DynamicMessageType(
  DynamicMessageTypeBuilder::SharedPtr dynamic_type_builder,
  rcl_allocator_t allocator)
: serialization_support_(dynamic_type_builder->get_shared_dynamic_serialization_support()),
  rosidl_dynamic_type_(rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type())
{
  if (!serialization_support_) {
    throw std::runtime_error("dynamic type could not bind serialization support!");
  }

  rosidl_dynamic_typesupport_dynamic_type_builder_t rosidl_dynamic_type_builder =
    dynamic_type_builder->get_rosidl_dynamic_type_builder();

  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_init_from_dynamic_type_builder(
    &rosidl_dynamic_type_builder, &allocator, &get_rosidl_dynamic_type());
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not init new dynamic type object");
  }
}


DynamicMessageType::DynamicMessageType(
  DynamicSerializationSupport::SharedPtr serialization_support,
  rosidl_dynamic_typesupport_dynamic_type_t && rosidl_dynamic_type)
: serialization_support_(serialization_support),
  rosidl_dynamic_type_(std::move(rosidl_dynamic_type))
{
  if (serialization_support) {
    if (!match_serialization_support_(*serialization_support, rosidl_dynamic_type)) {
      throw std::runtime_error(
              "serialization support library identifier does not match dynamic type's!");
    }
  }
}


DynamicMessageType::DynamicMessageType(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription & description,
  rcl_allocator_t allocator)
: serialization_support_(serialization_support),
  rosidl_dynamic_type_(rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type())
{
  init_from_description(description, allocator, serialization_support);
}


DynamicMessageType::DynamicMessageType(DynamicMessageType && other) noexcept
: serialization_support_(std::exchange(other.serialization_support_, nullptr)),
  rosidl_dynamic_type_(std::exchange(
      other.rosidl_dynamic_type_, rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type()))
{}


DynamicMessageType &
DynamicMessageType::operator=(DynamicMessageType && other) noexcept
{
  std::swap(serialization_support_, other.serialization_support_);
  std::swap(rosidl_dynamic_type_, other.rosidl_dynamic_type_);
  return *this;
}


DynamicMessageType::~DynamicMessageType()
{
  if (rosidl_dynamic_typesupport_dynamic_type_fini(&get_rosidl_dynamic_type()) != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR("could not fini rosidl dynamic type");
  }
}


void
DynamicMessageType::init_from_description(
  const rosidl_runtime_c__type_description__TypeDescription & description,
  rcl_allocator_t allocator,
  DynamicSerializationSupport::SharedPtr serialization_support)
{
  if (serialization_support) {
    // Swap serialization support if serialization support is given
    serialization_support_ = serialization_support;
  }

  rosidl_dynamic_typesupport_dynamic_type_t rosidl_dynamic_type =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type();
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_init_from_description(
    &serialization_support_->get_rosidl_serialization_support(),
    &description,
    &allocator,
    &rosidl_dynamic_type);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not init new dynamic type object");
  }

  rosidl_dynamic_type_ = std::move(rosidl_dynamic_type);
}


bool
DynamicMessageType::match_serialization_support_(
  const DynamicSerializationSupport & serialization_support,
  const rosidl_dynamic_typesupport_dynamic_type_t & rosidl_dynamic_type)
{
  if (serialization_support.get_serialization_library_identifier() != std::string(
      rosidl_dynamic_type.serialization_support->serialization_library_identifier))
  {
    RCUTILS_LOG_ERROR(
      "serialization support library identifier does not match dynamic type's (%s vs %s)",
      serialization_support.get_serialization_library_identifier().c_str(),
      rosidl_dynamic_type.serialization_support->serialization_library_identifier);
    return false;
  }
  return true;
}


// GETTERS =========================================================================================
const std::string
DynamicMessageType::get_serialization_library_identifier() const
{
  return std::string(
    get_rosidl_dynamic_type().serialization_support->serialization_library_identifier);
}


const std::string
DynamicMessageType::get_name() const
{
  size_t buf_length;
  const char * buf;
  rosidl_dynamic_typesupport_dynamic_type_get_name(&get_rosidl_dynamic_type(), &buf, &buf_length);
  return std::string(buf, buf_length);
}


size_t
DynamicMessageType::get_member_count() const
{
  size_t out;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_get_member_count(
    &get_rosidl_dynamic_type(), &out);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error(
            std::string("could not get member count: ") + rcl_get_error_string().str);
  }
  return out;
}


rosidl_dynamic_typesupport_dynamic_type_t &
DynamicMessageType::get_rosidl_dynamic_type()
{
  return rosidl_dynamic_type_;
}


const rosidl_dynamic_typesupport_dynamic_type_t &
DynamicMessageType::get_rosidl_dynamic_type() const
{
  return rosidl_dynamic_type_;
}


DynamicSerializationSupport::SharedPtr
DynamicMessageType::get_shared_dynamic_serialization_support()
{
  return serialization_support_;
}


DynamicSerializationSupport::ConstSharedPtr
DynamicMessageType::get_shared_dynamic_serialization_support() const
{
  return serialization_support_;
}


// METHODS =========================================================================================
DynamicMessageType
DynamicMessageType::clone(rcl_allocator_t allocator)
{
  rosidl_dynamic_typesupport_dynamic_type_t rosidl_dynamic_type =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type();
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_clone(
    &get_rosidl_dynamic_type(), &allocator, &rosidl_dynamic_type);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error(
            std::string("could not clone dynamic type: ") + rcl_get_error_string().str);
  }
  return DynamicMessageType(
    get_shared_dynamic_serialization_support(), std::move(rosidl_dynamic_type));
}


DynamicMessageType::SharedPtr
DynamicMessageType::clone_shared(rcl_allocator_t allocator)
{
  rosidl_dynamic_typesupport_dynamic_type_t rosidl_dynamic_type =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type();
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_clone(
    &get_rosidl_dynamic_type(), &allocator, &rosidl_dynamic_type);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error(
            std::string("could not clone dynamic type: ") + rcl_get_error_string().str);
  }
  return DynamicMessageType::make_shared(
    get_shared_dynamic_serialization_support(), std::move(rosidl_dynamic_type));
}


bool
DynamicMessageType::equals(const DynamicMessageType & other) const
{
  if (get_serialization_library_identifier() != other.get_serialization_library_identifier()) {
    throw std::runtime_error("library identifiers don't match");
  }
  bool out;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_equals(
    &get_rosidl_dynamic_type(), &other.get_rosidl_dynamic_type(), &out);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error(
            std::string("could not equate dynamic message types: ") + rcl_get_error_string().str);
  }
  return out;
}


DynamicMessage
DynamicMessageType::build_dynamic_message(rcl_allocator_t allocator)
{
  return DynamicMessage(shared_from_this(), allocator);
}


DynamicMessage::SharedPtr
DynamicMessageType::build_dynamic_message_shared(rcl_allocator_t allocator)
{
  return DynamicMessage::make_shared(shared_from_this(), allocator);
}
