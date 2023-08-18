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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_MESSAGE_TYPE_BUILDER_IMPL_HPP_
// Template specialization implementations
#include "rclcpp/dynamic_typesupport/detail/dynamic_message_type_builder_impl.hpp"
#endif

// CONSTRUCTION ====================================================================================
DynamicMessageTypeBuilder::DynamicMessageTypeBuilder(
  DynamicSerializationSupport::SharedPtr serialization_support, const std::string & name)
: DynamicMessageTypeBuilder::DynamicMessageTypeBuilder(
    serialization_support,
    name,
    serialization_support->get_rosidl_serialization_support().allocator) {}


DynamicMessageTypeBuilder::DynamicMessageTypeBuilder(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const std::string & name,
  rcl_allocator_t allocator)
: serialization_support_(serialization_support),
  rosidl_dynamic_type_builder_(
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder())
{
  init_from_serialization_support_(serialization_support, name, allocator);
}


DynamicMessageTypeBuilder::DynamicMessageTypeBuilder(
  DynamicSerializationSupport::SharedPtr serialization_support,
  rosidl_dynamic_typesupport_dynamic_type_builder_t && rosidl_dynamic_type_builder)
: serialization_support_(serialization_support),
  rosidl_dynamic_type_builder_(
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder())
{
  if (!serialization_support) {
    throw std::runtime_error("serialization support cannot be nullptr!");
  }
  if (!match_serialization_support_(*serialization_support, rosidl_dynamic_type_builder)) {
    throw std::runtime_error(
            "serialization support library does not match dynamic type builder's!");
  }
}


DynamicMessageTypeBuilder::DynamicMessageTypeBuilder(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription & description,
  rcl_allocator_t allocator)
: serialization_support_(serialization_support),
  rosidl_dynamic_type_builder_(
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder())
{
  if (!serialization_support) {
    throw std::runtime_error("serialization support cannot be nullptr!");
  }
  init_from_description(description, allocator, serialization_support);
}


DynamicMessageTypeBuilder::DynamicMessageTypeBuilder(DynamicMessageTypeBuilder && other) noexcept
: serialization_support_(std::exchange(other.serialization_support_, nullptr)),
  rosidl_dynamic_type_builder_(std::exchange(
      other.rosidl_dynamic_type_builder_,
      rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder())) {}


DynamicMessageTypeBuilder &
DynamicMessageTypeBuilder::operator=(DynamicMessageTypeBuilder && other) noexcept
{
  std::swap(serialization_support_, other.serialization_support_);
  std::swap(rosidl_dynamic_type_builder_, other.rosidl_dynamic_type_builder_);
  return *this;
}


DynamicMessageTypeBuilder::~DynamicMessageTypeBuilder()
{
  if (rosidl_dynamic_typesupport_dynamic_type_builder_fini(&get_rosidl_dynamic_type_builder()) !=
    RCUTILS_RET_OK)
  {
    RCUTILS_LOG_ERROR("could not fini rosidl dynamic type builder");
  }
}


void
DynamicMessageTypeBuilder::init_from_description(
  const rosidl_runtime_c__type_description__TypeDescription & description,
  rcl_allocator_t allocator,
  DynamicSerializationSupport::SharedPtr serialization_support)
{
  if (serialization_support) {
    // Swap serialization support if serialization support is given
    serialization_support_ = serialization_support;
  }

  rosidl_dynamic_typesupport_dynamic_type_builder_t rosidl_dynamic_type_builder =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder();
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_builder_init_from_description(
    &serialization_support_->get_rosidl_serialization_support(),
    &description,
    &allocator,
    &rosidl_dynamic_type_builder);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not init new dynamic type builder object");
  }

  rosidl_dynamic_type_builder_ = std::move(rosidl_dynamic_type_builder);
}


void
DynamicMessageTypeBuilder::init_from_serialization_support_(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const std::string & name,
  rcl_allocator_t allocator)
{
  if (!serialization_support) {
    throw std::runtime_error("serialization support cannot be nullptr!");
  }
  if (!&serialization_support->get_rosidl_serialization_support()) {
    throw std::runtime_error("serialization support raw pointer cannot be nullptr!");
  }

  rosidl_dynamic_typesupport_dynamic_type_builder_t rosidl_dynamic_type_builder =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder();
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_builder_init(
    &serialization_support->get_rosidl_serialization_support(),
    name.c_str(), name.size(),
    &allocator,
    &rosidl_dynamic_type_builder);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error(
            std::string("could not init dynamic type builder: ") + rcl_get_error_string().str);
  }
}


bool
DynamicMessageTypeBuilder::match_serialization_support_(
  const DynamicSerializationSupport & serialization_support,
  const rosidl_dynamic_typesupport_dynamic_type_builder_t & rosidl_dynamic_type_builder)
{
  if (serialization_support.get_serialization_library_identifier() != std::string(
      rosidl_dynamic_type_builder.serialization_support->serialization_library_identifier))
  {
    RCUTILS_LOG_ERROR(
      "serialization support library identifier does not match dynamic type builder's");
    return false;
  }
  return true;
}


// GETTERS =======================================================================================
const std::string
DynamicMessageTypeBuilder::get_serialization_library_identifier() const
{
  return std::string(
    get_rosidl_dynamic_type_builder().serialization_support->serialization_library_identifier);
}


const std::string
DynamicMessageTypeBuilder::get_name() const
{
  size_t buf_length;
  const char * buf;
  rosidl_dynamic_typesupport_dynamic_type_builder_get_name(
    &get_rosidl_dynamic_type_builder(), &buf, &buf_length);
  return std::string(buf, buf_length);
}


rosidl_dynamic_typesupport_dynamic_type_builder_t &
DynamicMessageTypeBuilder::get_rosidl_dynamic_type_builder()
{
  return rosidl_dynamic_type_builder_;
}


const rosidl_dynamic_typesupport_dynamic_type_builder_t &
DynamicMessageTypeBuilder::get_rosidl_dynamic_type_builder() const
{
  return rosidl_dynamic_type_builder_;
}


DynamicSerializationSupport::SharedPtr
DynamicMessageTypeBuilder::get_shared_dynamic_serialization_support()
{
  return serialization_support_;
}


DynamicSerializationSupport::ConstSharedPtr
DynamicMessageTypeBuilder::get_shared_dynamic_serialization_support() const
{
  return serialization_support_;
}


// METHODS =======================================================================================
void
DynamicMessageTypeBuilder::set_name(const std::string & name)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_set_name(
    &get_rosidl_dynamic_type_builder(), name.c_str(), name.size());
}


DynamicMessageTypeBuilder
DynamicMessageTypeBuilder::clone(rcl_allocator_t allocator)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_t rosidl_dynamic_type_builder =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder();
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_builder_clone(
    &get_rosidl_dynamic_type_builder(), &allocator, &rosidl_dynamic_type_builder);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error(
            std::string("could not clone dynamic type builder: ") + rcl_get_error_string().str);
  }
  return DynamicMessageTypeBuilder(
    get_shared_dynamic_serialization_support(), std::move(rosidl_dynamic_type_builder));
}


DynamicMessageTypeBuilder::SharedPtr
DynamicMessageTypeBuilder::clone_shared(rcl_allocator_t allocator)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_t rosidl_dynamic_type_builder =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder();
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_type_builder_clone(
    &get_rosidl_dynamic_type_builder(), &allocator, &rosidl_dynamic_type_builder);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error(
            std::string("could not clone dynamic type builder: ") + rcl_get_error_string().str);
  }
  return DynamicMessageTypeBuilder::make_shared(
    get_shared_dynamic_serialization_support(), std::move(rosidl_dynamic_type_builder));
}


void
DynamicMessageTypeBuilder::clear(rcl_allocator_t allocator)
{
  if (!serialization_support_) {
    throw std::runtime_error(
            "cannot call clear() on a dynamic type builder with uninitialized serialization support"
    );
  }

  const std::string & name = get_name();
  init_from_serialization_support_(serialization_support_, name, allocator);
}


DynamicMessage
DynamicMessageTypeBuilder::build_dynamic_message(rcl_allocator_t allocator)
{
  return DynamicMessage(shared_from_this(), allocator);
}


DynamicMessage::SharedPtr
DynamicMessageTypeBuilder::build_dynamic_message_shared(rcl_allocator_t allocator)
{
  return DynamicMessage::make_shared(shared_from_this(), allocator);
}


DynamicMessageType
DynamicMessageTypeBuilder::build_dynamic_message_type(rcl_allocator_t allocator)
{
  return DynamicMessageType(shared_from_this(), allocator);
}


DynamicMessageType::SharedPtr
DynamicMessageTypeBuilder::build_dynamic_message_type_shared(rcl_allocator_t allocator)
{
  return DynamicMessageType::make_shared(shared_from_this(), allocator);
}


// ADD MEMBERS =====================================================================================
// Defined in "detail/dynamic_message_type_builder_impl.hpp"


// ADD FIXED STRING MEMBERS ========================================================================
void
DynamicMessageTypeBuilder::add_fixed_string_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_length,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    string_length);
}


void
DynamicMessageTypeBuilder::add_fixed_wstring_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_length,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    wstring_length);
}


void
DynamicMessageTypeBuilder::add_fixed_string_array_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t string_length, size_t array_length, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_array_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    string_length, array_length);
}


void
DynamicMessageTypeBuilder::add_fixed_wstring_array_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t wstring_length, size_t array_length, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_array_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    wstring_length, array_length);
}


void
DynamicMessageTypeBuilder::add_fixed_string_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_length,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_unbounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    string_length);
}


void
DynamicMessageTypeBuilder::add_fixed_wstring_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_length,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_unbounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    wstring_length);
}


void
DynamicMessageTypeBuilder::add_fixed_string_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t string_length, size_t sequence_bound, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_bounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    string_length, sequence_bound);
}


void
DynamicMessageTypeBuilder::add_fixed_wstring_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t wstring_length, size_t sequence_bound, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_bounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    wstring_length, sequence_bound);
}


// ADD BOUNDED STRING MEMBERS ======================================================================
void
DynamicMessageTypeBuilder::add_bounded_string_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_bound,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    string_bound);
}


void
DynamicMessageTypeBuilder::add_bounded_wstring_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_bound,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    wstring_bound);
}


void
DynamicMessageTypeBuilder::add_bounded_string_array_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t string_bound, size_t array_length, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_array_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    string_bound, array_length);
}


void
DynamicMessageTypeBuilder::add_bounded_wstring_array_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t wstring_bound, size_t array_length, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_array_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    wstring_bound, array_length);
}


void
DynamicMessageTypeBuilder::add_bounded_string_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_bound,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_unbounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    string_bound);
}


void
DynamicMessageTypeBuilder::add_bounded_wstring_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_bound,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_unbounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    wstring_bound);
}


void
DynamicMessageTypeBuilder::add_bounded_string_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t string_bound, size_t sequence_bound, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_bounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    string_bound, sequence_bound);
}


void
DynamicMessageTypeBuilder::add_bounded_wstring_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t wstring_bound, size_t sequence_bound, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_bounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    wstring_bound, sequence_bound);
}


// ADD NESTED MEMBERS ==============================================================================
void
DynamicMessageTypeBuilder::add_complex_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicMessageType & nested_type, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    &nested_type.get_rosidl_dynamic_type());
}


void
DynamicMessageTypeBuilder::add_complex_array_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicMessageType & nested_type, size_t array_length, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_array_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    &nested_type.get_rosidl_dynamic_type(), array_length);
}


void
DynamicMessageTypeBuilder::add_complex_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicMessageType & nested_type, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_unbounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    &nested_type.get_rosidl_dynamic_type());
}


void
DynamicMessageTypeBuilder::add_complex_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicMessageType & nested_type, size_t sequence_bound, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_bounded_sequence_member(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    &nested_type.get_rosidl_dynamic_type(), sequence_bound);
}


void
DynamicMessageTypeBuilder::add_complex_member_builder(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicMessageTypeBuilder & nested_type_builder, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_member_builder(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    &nested_type_builder.get_rosidl_dynamic_type_builder());
}


void
DynamicMessageTypeBuilder::add_complex_array_member_builder(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicMessageTypeBuilder & nested_type_builder, size_t array_length,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_array_member_builder(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    &nested_type_builder.get_rosidl_dynamic_type_builder(), array_length);
}


void
DynamicMessageTypeBuilder::add_complex_unbounded_sequence_member_builder(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicMessageTypeBuilder & nested_type_builder, const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_unbounded_sequence_member_builder(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    &nested_type_builder.get_rosidl_dynamic_type_builder());
}


void
DynamicMessageTypeBuilder::add_complex_bounded_sequence_member_builder(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicMessageTypeBuilder & nested_type_builder, size_t sequence_bound,
  const std::string & default_value)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_bounded_sequence_member_builder(
    &get_rosidl_dynamic_type_builder(),
    id, name.c_str(), name.size(), default_value.c_str(), default_value.size(),
    &nested_type_builder.get_rosidl_dynamic_type_builder(), sequence_bound);
}
