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


// CONSTRUCTION ====================================================================================
DynamicType::DynamicType(DynamicTypeBuilder::SharedPtr dynamic_type_builder)
: serialization_support_(dynamic_type_builder->get_shared_dynamic_serialization_support()),
  rosidl_dynamic_type_(nullptr)
{
  if (!serialization_support_) {
    throw std::runtime_error("dynamic type could not bind serialization support!");
  }

  rosidl_dynamic_typesupport_dynamic_type_builder_t * rosidl_dynamic_type_builder =
    dynamic_type_builder->get_rosidl_dynamic_type_builder();
  if (!rosidl_dynamic_type_builder) {
    throw std::runtime_error("dynamic type builder cannot be nullptr!");
  }

  rosidl_dynamic_typesupport_dynamic_type_t * rosidl_dynamic_type = nullptr;
  rosidl_dynamic_type = rosidl_dynamic_typesupport_dynamic_type_init_from_dynamic_type_builder(
    rosidl_dynamic_type_builder);
  if (!rosidl_dynamic_type) {
    throw std::runtime_error("could not create new dynamic type object");
  }

  rosidl_dynamic_type_.reset(
    rosidl_dynamic_type,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_type_t * rosidl_dynamic_type)->void {
      rosidl_dynamic_typesupport_dynamic_type_fini(rosidl_dynamic_type);
      free(rosidl_dynamic_type);
    });
}


DynamicType::DynamicType(
  DynamicSerializationSupport::SharedPtr serialization_support,
  rosidl_dynamic_typesupport_dynamic_type_t * rosidl_dynamic_type)
: serialization_support_(serialization_support), rosidl_dynamic_type_(nullptr)
{
  if (!rosidl_dynamic_type) {
    throw std::runtime_error("rosidl dynamic type cannot be nullptr!");
  }
  if (serialization_support) {
    if (!match_serialization_support_(*serialization_support, *rosidl_dynamic_type)) {
      throw std::runtime_error(
              "serialization support library identifier does not match dynamic type's!");
    }
  }

  rosidl_dynamic_type_.reset(
    rosidl_dynamic_type,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_type_t * rosidl_dynamic_type)->void {
      rosidl_dynamic_typesupport_dynamic_type_fini(rosidl_dynamic_type);
      free(rosidl_dynamic_type);
    });
}


DynamicType::DynamicType(
  DynamicSerializationSupport::SharedPtr serialization_support,
  std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_t> rosidl_dynamic_type)
: serialization_support_(serialization_support), rosidl_dynamic_type_(rosidl_dynamic_type)
{
  if (!rosidl_dynamic_type) {
    throw std::runtime_error("rosidl dynamic type cannot be nullptr!");
  }
  if (serialization_support) {
    if (!match_serialization_support_(*serialization_support, *rosidl_dynamic_type)) {
      throw std::runtime_error(
              "serialization support library identifier does not match dynamic type's!");
    }
  }
}


DynamicType::DynamicType(
  DynamicSerializationSupport::SharedPtr serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription * description)
: serialization_support_(serialization_support), rosidl_dynamic_type_(nullptr)
{
  init_from_description(description, serialization_support);
}


DynamicType::DynamicType(const DynamicType & other)
: enable_shared_from_this(), serialization_support_(nullptr), rosidl_dynamic_type_(nullptr)
{
  DynamicType out = other.clone();
  std::swap(serialization_support_, out.serialization_support_);
  std::swap(rosidl_dynamic_type_, out.rosidl_dynamic_type_);
}


DynamicType::DynamicType(DynamicType && other) noexcept
: serialization_support_(std::exchange(other.serialization_support_, nullptr)),
  rosidl_dynamic_type_(std::exchange(other.rosidl_dynamic_type_, nullptr)) {}


DynamicType &
DynamicType::operator=(const DynamicType & other)
{
  return *this = DynamicType(other);
}


DynamicType &
DynamicType::operator=(DynamicType && other) noexcept
{
  std::swap(serialization_support_, other.serialization_support_);
  std::swap(rosidl_dynamic_type_, other.rosidl_dynamic_type_);
  return *this;
}


DynamicType::~DynamicType() {}


void
DynamicType::init_from_description(
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

  rosidl_dynamic_typesupport_dynamic_type_t * rosidl_dynamic_type = nullptr;
  rosidl_dynamic_type =
    rosidl_dynamic_typesupport_dynamic_type_init_from_description(
    serialization_support_->get_rosidl_serialization_support(), description);
  if (!rosidl_dynamic_type) {
    throw std::runtime_error("could not create new dynamic type object");
  }

  rosidl_dynamic_type_.reset(
    rosidl_dynamic_type,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_type_t * rosidl_dynamic_type)->void {
      rosidl_dynamic_typesupport_dynamic_type_fini(rosidl_dynamic_type);
      free(rosidl_dynamic_type);
    });
}


bool
DynamicType::match_serialization_support_(
  const DynamicSerializationSupport & serialization_support,
  const rosidl_dynamic_typesupport_dynamic_type_t & rosidl_dynamic_type)
{
  bool out = true;

  if (serialization_support.get_library_identifier() != std::string(
      rosidl_dynamic_type.serialization_support->library_identifier))
  {
    RCUTILS_LOG_ERROR(
      "serialization support library identifier does not match dynamic type's");
    out = false;
  }

  // TODO(methylDragon): Can I do this?? Is it portable?
  if (serialization_support.get_rosidl_serialization_support() !=
      rosidl_dynamic_type.serialization_support)
  {
    RCUTILS_LOG_ERROR(
      "serialization support pointer does not match dynamic type's");
    out = false;
  }

  return out;
}


// GETTERS =========================================================================================
const std::string
DynamicType::get_library_identifier() const
{
  return std::string(rosidl_dynamic_type_->serialization_support->library_identifier);
}


const std::string
DynamicType::get_name() const
{
  size_t buf_length;
  const char * buf = rosidl_dynamic_typesupport_dynamic_type_get_name(
    get_rosidl_dynamic_type(), &buf_length);
  return std::string(buf, buf_length);
}


size_t
DynamicType::get_member_count() const
{
  return rosidl_dynamic_typesupport_dynamic_type_get_member_count(rosidl_dynamic_type_.get());
}


rosidl_dynamic_typesupport_dynamic_type_t *
DynamicType::get_rosidl_dynamic_type()
{
  return rosidl_dynamic_type_.get();
}


const rosidl_dynamic_typesupport_dynamic_type_t *
DynamicType::get_rosidl_dynamic_type() const
{
  return rosidl_dynamic_type_.get();
}


std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_t>
DynamicType::get_shared_rosidl_dynamic_type()
{
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_t>(
    shared_from_this(), rosidl_dynamic_type_.get());
}


std::shared_ptr<const rosidl_dynamic_typesupport_dynamic_type_t>
DynamicType::get_shared_rosidl_dynamic_type() const
{
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_type_t>(
    shared_from_this(), rosidl_dynamic_type_.get());
}


DynamicSerializationSupport::SharedPtr
DynamicType::get_shared_dynamic_serialization_support()
{
  return serialization_support_;
}


DynamicSerializationSupport::ConstSharedPtr
DynamicType::get_shared_dynamic_serialization_support() const
{
  return serialization_support_;
}


// METHODS =========================================================================================
DynamicType
DynamicType::clone() const
{
  return DynamicType(
    serialization_support_,
    rosidl_dynamic_typesupport_dynamic_type_clone(get_rosidl_dynamic_type()));
}


DynamicType::SharedPtr
DynamicType::clone_shared() const
{
  return DynamicType::make_shared(
    serialization_support_,
    rosidl_dynamic_typesupport_dynamic_type_clone(get_rosidl_dynamic_type()));
}


bool
DynamicType::equals(const DynamicType & other) const
{
  if (get_library_identifier() != other.get_library_identifier()) {
    throw std::runtime_error("library identifiers don't match");
  }
  return rosidl_dynamic_typesupport_dynamic_type_equals(
    get_rosidl_dynamic_type(), other.get_rosidl_dynamic_type());
}


DynamicData
DynamicType::build_data()
{
  return DynamicData(shared_from_this());
}


DynamicData::SharedPtr
DynamicType::build_data_shared()
{
  return DynamicData::make_shared(shared_from_this());
}
