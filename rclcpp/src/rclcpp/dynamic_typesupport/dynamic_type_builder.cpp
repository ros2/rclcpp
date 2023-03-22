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

#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_data.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_type_builder.hpp"
#include "rclcpp/exceptions.hpp"
#include "rcutils/logging_macros.h"

#include <rosidl_dynamic_typesupport/api/dynamic_data.h>
#include <rosidl_dynamic_typesupport/api/dynamic_type.h>
#include <rosidl_dynamic_typesupport/types.h>

using rclcpp::dynamic_typesupport::DynamicData;
using rclcpp::dynamic_typesupport::DynamicSerializationSupport;
using rclcpp::dynamic_typesupport::DynamicType;
using rclcpp::dynamic_typesupport::DynamicTypeBuilder;


#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_TYPE_BUILDER_IMPL_HPP_
// Template specialization implementations
#include "rclcpp/dynamic_typesupport/detail/dynamic_type_builder_impl.hpp"
#endif


// CONSTRUCTION ==================================================================================
DynamicTypeBuilder::DynamicTypeBuilder(
  DynamicSerializationSupport::SharedPtr serialization_support, const std::string & name)
: serialization_support_(serialization_support), rosidl_dynamic_type_builder_(nullptr)
{
  init_from_serialization_support_(serialization_support, name);
  if (!rosidl_dynamic_type_builder_) {
    throw std::runtime_error("could not create new dynamic type builder object");
  }
}


DynamicTypeBuilder::DynamicTypeBuilder(
  DynamicSerializationSupport::SharedPtr serialization_support,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * rosidl_dynamic_type_builder)
: serialization_support_(serialization_support), rosidl_dynamic_type_builder_(nullptr)
{
  if (!serialization_support) {
    throw std::runtime_error("serialization support cannot be nullptr!");
  }
  if (!rosidl_dynamic_type_builder) {
    throw std::runtime_error("rosidl dynamic type builder cannot be nullptr!");
  }
  if (!match_serialization_support_(*serialization_support, *rosidl_dynamic_type_builder)) {
    throw std::runtime_error(
            "serialization support library does not match dynamic type builder's!");
  }

  rosidl_dynamic_type_builder_.reset(
    rosidl_dynamic_type_builder,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_type_builder_t * rosidl_dynamic_type_builder)->void {
      rosidl_dynamic_typesupport_dynamic_type_builder_fini(rosidl_dynamic_type_builder);
      free(rosidl_dynamic_type_builder);
    });
}


DynamicTypeBuilder::DynamicTypeBuilder(
  DynamicSerializationSupport::SharedPtr serialization_support,
  std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_builder_t> rosidl_dynamic_type_builder)
: serialization_support_(serialization_support),
  rosidl_dynamic_type_builder_(rosidl_dynamic_type_builder)
{
  if (!serialization_support) {
    throw std::runtime_error("serialization support cannot be nullptr!");
  }
  if (!rosidl_dynamic_type_builder) {
    throw std::runtime_error("rosidl dynamic type builder cannot be nullptr!");
  }
  if (!match_serialization_support_(*serialization_support, *rosidl_dynamic_type_builder.get())) {
    throw std::runtime_error(
            "serialization support library does not match dynamic type builder's!");
  }
}


DynamicTypeBuilder::DynamicTypeBuilder(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription * description)
: serialization_support_(serialization_support),
  rosidl_dynamic_type_builder_(nullptr)
{
  if (!serialization_support) {
    throw std::runtime_error("serialization support cannot be nullptr!");
  }
  init_from_description(description, serialization_support);
}


DynamicTypeBuilder::DynamicTypeBuilder(const DynamicTypeBuilder & other)
: enable_shared_from_this(), serialization_support_(nullptr), rosidl_dynamic_type_builder_(nullptr)
{
  DynamicTypeBuilder out = other.clone();
  std::swap(serialization_support_, out.serialization_support_);
  std::swap(rosidl_dynamic_type_builder_, out.rosidl_dynamic_type_builder_);
}


DynamicTypeBuilder::DynamicTypeBuilder(DynamicTypeBuilder && other) noexcept
: serialization_support_(std::exchange(other.serialization_support_, nullptr)),
  rosidl_dynamic_type_builder_(std::exchange(other.rosidl_dynamic_type_builder_, nullptr)) {}


DynamicTypeBuilder &
DynamicTypeBuilder::operator=(const DynamicTypeBuilder & other)
{
  return *this = DynamicTypeBuilder(other);
}


DynamicTypeBuilder &
DynamicTypeBuilder::operator=(DynamicTypeBuilder && other) noexcept
{
  std::swap(serialization_support_, other.serialization_support_);
  std::swap(rosidl_dynamic_type_builder_, other.rosidl_dynamic_type_builder_);
  return *this;
}


DynamicTypeBuilder::~DynamicTypeBuilder() {}


void
DynamicTypeBuilder::init_from_description(
  const rosidl_runtime_c__type_description__TypeDescription * description,
  DynamicSerializationSupport::SharedPtr serialization_support)
{
  if (!description) {
    throw std::runtime_error("description cannot be nullptr!");
  }
  if (serialization_support) {
    // Swap serialization support if serialization support is given
    serialization_support_ = serialization_support;
  }

  rosidl_dynamic_typesupport_dynamic_type_builder_t * rosidl_dynamic_type_builder = nullptr;
  rosidl_dynamic_type_builder =
    rosidl_dynamic_typesupport_dynamic_type_builder_init_from_description(
    serialization_support_->get_rosidl_serialization_support(), description);
  if (!rosidl_dynamic_type_builder) {
    throw std::runtime_error("could not create new dynamic type builder object");
  }

  rosidl_dynamic_type_builder_.reset(
    rosidl_dynamic_type_builder,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_type_builder_t * rosidl_dynamic_type_builder)->void {
      rosidl_dynamic_typesupport_dynamic_type_builder_fini(rosidl_dynamic_type_builder);
      free(rosidl_dynamic_type_builder);
    });
}


void
DynamicTypeBuilder::init_from_serialization_support_(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const std::string & name)
{
  if (!serialization_support) {
    throw std::runtime_error("serialization support cannot be nullptr!");
  }
  if (!serialization_support->get_rosidl_serialization_support()) {
    throw std::runtime_error("serialization support raw pointer cannot be nullptr!");
  }


  rosidl_dynamic_typesupport_dynamic_type_builder_t * rosidl_dynamic_type_builder = nullptr;
  rosidl_dynamic_type_builder = rosidl_dynamic_typesupport_dynamic_type_builder_init(
    serialization_support->get_rosidl_serialization_support(), name.c_str(), name.size());

  if (!rosidl_dynamic_type_builder) {
    throw std::runtime_error("could not create new dynamic type builder object");
  }

  rosidl_dynamic_type_builder_.reset(
    rosidl_dynamic_type_builder,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_type_builder_t * rosidl_dynamic_type_builder)->void {
      rosidl_dynamic_typesupport_dynamic_type_builder_fini(rosidl_dynamic_type_builder);
      free(rosidl_dynamic_type_builder);
    });
}


bool
DynamicTypeBuilder::match_serialization_support_(
  const DynamicSerializationSupport & serialization_support,
  const rosidl_dynamic_typesupport_dynamic_type_builder_t & rosidl_dynamic_type_builder)
{
  bool out = true;

  if (serialization_support.get_library_identifier() != std::string(
      rosidl_dynamic_type_builder.serialization_support->library_identifier))
  {
    RCUTILS_LOG_ERROR(
      "serialization support library identifier does not match dynamic type builder's");
    out = false;
  }

  // TODO(methylDragon): Can I do this?? Is it portable?
  if (serialization_support.get_rosidl_serialization_support() !=
    rosidl_dynamic_type_builder.serialization_support)
  {
    RCUTILS_LOG_ERROR(
      "serialization support pointer does not match dynamic type builder's");
    out = false;
  }

  return out;
}


// GETTERS =======================================================================================
const std::string
DynamicTypeBuilder::get_library_identifier() const
{
  return std::string(rosidl_dynamic_type_builder_->serialization_support->library_identifier);
}


const std::string
DynamicTypeBuilder::get_name() const
{
  size_t buf_length;
  const char * buf = rosidl_dynamic_typesupport_dynamic_type_builder_get_name(
    get_rosidl_dynamic_type_builder(), &buf_length);
  return std::string(buf, buf_length);
}


rosidl_dynamic_typesupport_dynamic_type_builder_t *
DynamicTypeBuilder::get_rosidl_dynamic_type_builder()
{
  return rosidl_dynamic_type_builder_.get();
}


const rosidl_dynamic_typesupport_dynamic_type_builder_t *
DynamicTypeBuilder::get_rosidl_dynamic_type_builder() const
{
  return rosidl_dynamic_type_builder_.get();
}


std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_builder_t>
DynamicTypeBuilder::get_shared_rosidl_dynamic_type_builder()
{
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_builder_t>(
    shared_from_this(), rosidl_dynamic_type_builder_.get());
}


std::shared_ptr<const rosidl_dynamic_typesupport_dynamic_type_builder_t>
DynamicTypeBuilder::get_shared_rosidl_dynamic_type_builder() const
{
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_builder_t>(
    shared_from_this(), rosidl_dynamic_type_builder_.get());
}


DynamicSerializationSupport::SharedPtr
DynamicTypeBuilder::get_shared_dynamic_serialization_support()
{
  return serialization_support_;
}


DynamicSerializationSupport::ConstSharedPtr
DynamicTypeBuilder::get_shared_dynamic_serialization_support() const
{
  return serialization_support_;
}


// METHODS =======================================================================================
void
DynamicTypeBuilder::set_name(const std::string & name)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_set_name(
    get_rosidl_dynamic_type_builder(), name.c_str(), name.size());
}


DynamicTypeBuilder
DynamicTypeBuilder::clone() const
{
  return DynamicTypeBuilder(
    serialization_support_,
    rosidl_dynamic_typesupport_dynamic_type_builder_clone(get_rosidl_dynamic_type_builder()));
}


DynamicTypeBuilder::SharedPtr
DynamicTypeBuilder::clone_shared() const
{
  return DynamicTypeBuilder::make_shared(
    serialization_support_,
    rosidl_dynamic_typesupport_dynamic_type_builder_clone(get_rosidl_dynamic_type_builder()));
}


void
DynamicTypeBuilder::clear()
{
  if (!serialization_support_) {
    throw std::runtime_error(
            "cannot call clear() on a dynamic type builder with uninitialized serialization support");
  }

  const std::string & name = get_name();
  init_from_serialization_support_(serialization_support_, name);
  if (!rosidl_dynamic_type_builder_) {
    throw std::runtime_error("could not create new dynamic type builder object");
  }
}


DynamicData
DynamicTypeBuilder::build_data()
{
  return DynamicData(shared_from_this());
}


DynamicData::SharedPtr
DynamicTypeBuilder::build_data_shared()
{
  return DynamicData::make_shared(shared_from_this());
}


DynamicType
DynamicTypeBuilder::build_type()
{
  return DynamicType(shared_from_this());
}


DynamicType::SharedPtr
DynamicTypeBuilder::build_type_shared()
{
  return DynamicType::make_shared(shared_from_this());
}


// ADD MEMBERS =====================================================================================
// Defined in "detail/dynamic_type_builder_impl.hpp"


// ADD BOUNDED STRING MEMBERS ======================================================================
void
DynamicTypeBuilder::add_bounded_string_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_bound)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(), string_bound);
}


void
DynamicTypeBuilder::add_bounded_wstring_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_bound)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(), wstring_bound);
}


void
DynamicTypeBuilder::add_bounded_string_array_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t string_bound, size_t array_length)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_array_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(), string_bound, array_length);
}


void
DynamicTypeBuilder::add_bounded_wstring_array_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t wstring_bound, size_t array_length)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_array_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(), wstring_bound, array_length);
}


void
DynamicTypeBuilder::add_bounded_string_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t string_bound)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_unbounded_sequence_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(), string_bound);
}


void
DynamicTypeBuilder::add_bounded_wstring_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name, size_t wstring_bound)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_unbounded_sequence_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(), wstring_bound);
}


void
DynamicTypeBuilder::add_bounded_string_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t string_bound, size_t sequence_bound)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_bounded_sequence_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    string_bound, sequence_bound);
}


void
DynamicTypeBuilder::add_bounded_wstring_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  size_t wstring_bound, size_t sequence_bound)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_bounded_sequence_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    wstring_bound, sequence_bound);
}


// ADD NESTED MEMBERS ==============================================================================
void
DynamicTypeBuilder::add_complex_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicType & nested_type)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    nested_type.get_rosidl_dynamic_type());
}


void
DynamicTypeBuilder::add_complex_array_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicType & nested_type, size_t array_length)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_array_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    nested_type.get_rosidl_dynamic_type(), array_length);
}


void
DynamicTypeBuilder::add_complex_unbounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicType & nested_type)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_unbounded_sequence_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    nested_type.get_rosidl_dynamic_type());
}


void
DynamicTypeBuilder::add_complex_bounded_sequence_member(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicType & nested_type, size_t sequence_bound)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_bounded_sequence_member(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    nested_type.get_rosidl_dynamic_type(), sequence_bound);
}


void
DynamicTypeBuilder::add_complex_member_builder(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicTypeBuilder & nested_type_builder)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_member_builder(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    nested_type_builder.get_rosidl_dynamic_type_builder());
}


void
DynamicTypeBuilder::add_complex_array_member_builder(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicTypeBuilder & nested_type_builder, size_t array_length)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_array_member_builder(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    nested_type_builder.get_rosidl_dynamic_type_builder(), array_length);
}


void
DynamicTypeBuilder::add_complex_unbounded_sequence_member_builder(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicTypeBuilder & nested_type_builder)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_unbounded_sequence_member_builder(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    nested_type_builder.get_rosidl_dynamic_type_builder());
}


void
DynamicTypeBuilder::add_complex_bounded_sequence_member_builder(
  rosidl_dynamic_typesupport_member_id_t id, const std::string & name,
  DynamicTypeBuilder & nested_type_builder, size_t sequence_bound)
{
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_bounded_sequence_member_builder(
    get_rosidl_dynamic_type_builder(), id, name.c_str(), name.size(),
    nested_type_builder.get_rosidl_dynamic_type_builder(), sequence_bound);
}
