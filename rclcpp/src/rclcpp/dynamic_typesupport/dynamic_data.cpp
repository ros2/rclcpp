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

#include <rosidl_dynamic_typesupport/api/dynamic_type.h>
#include <rosidl_dynamic_typesupport/api/dynamic_data.h>
#include <rosidl_dynamic_typesupport/types.h>

using rclcpp::dynamic_typesupport::DynamicData;
using rclcpp::dynamic_typesupport::DynamicSerializationSupport;
using rclcpp::dynamic_typesupport::DynamicType;
using rclcpp::dynamic_typesupport::DynamicTypeBuilder;

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_DATA_IMPL_HPP_
// Template specialization implementations
#include "rclcpp/dynamic_typesupport/detail/dynamic_data_impl.hpp"
#endif


// CONSTRUCTION ==================================================================================
DynamicData::DynamicData(const DynamicTypeBuilder::SharedPtr dynamic_type_builder)
: serialization_support_(dynamic_type_builder->get_shared_dynamic_serialization_support()),
  rosidl_dynamic_data_(nullptr),
  is_loaned_(false),
  parent_data_(nullptr)
{
  if (!serialization_support_) {
    throw std::runtime_error("dynamic type could not bind serialization support!");
  }

  rosidl_dynamic_typesupport_dynamic_type_builder_t * rosidl_dynamic_type_builder =
    dynamic_type_builder->get_rosidl_dynamic_type_builder();
  if (!rosidl_dynamic_type_builder) {
    throw std::runtime_error("dynamic type builder cannot be nullptr!");
  }

  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data = nullptr;
  rosidl_dynamic_data = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type_builder(
    rosidl_dynamic_type_builder);
  if (!rosidl_dynamic_data) {
    throw std::runtime_error("could not create new dynamic data object");
  }

  rosidl_dynamic_data_.reset(
    rosidl_dynamic_data,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data)->void {
      rosidl_dynamic_typesupport_dynamic_data_fini(rosidl_dynamic_data);
      free(rosidl_dynamic_data);
    });
}


DynamicData::DynamicData(const DynamicType::SharedPtr dynamic_type)
: serialization_support_(dynamic_type->get_shared_dynamic_serialization_support()),
  rosidl_dynamic_data_(nullptr),
  is_loaned_(false),
  parent_data_(nullptr)
{
  if (!serialization_support_) {
    throw std::runtime_error("dynamic type could not bind serialization support!");
  }

  rosidl_dynamic_typesupport_dynamic_type_t * rosidl_dynamic_type =
    dynamic_type->get_rosidl_dynamic_type();
  if (!rosidl_dynamic_type) {
    throw std::runtime_error("dynamic type cannot be nullptr!");
  }

  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data = nullptr;
  rosidl_dynamic_data = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type(
    rosidl_dynamic_type);
  if (!rosidl_dynamic_data) {
    throw std::runtime_error("could not create new dynamic data object");
  }

  rosidl_dynamic_data_.reset(
    rosidl_dynamic_data,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data)->void {
      rosidl_dynamic_typesupport_dynamic_data_fini(rosidl_dynamic_data);
      free(rosidl_dynamic_data);
    });
}


DynamicData::DynamicData(
  DynamicSerializationSupport::SharedPtr serialization_support,
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data)
: serialization_support_(serialization_support),
  rosidl_dynamic_data_(nullptr),
  is_loaned_(false),
  parent_data_(nullptr)
{
  if (!rosidl_dynamic_data) {
    throw std::runtime_error("rosidl dynamic data cannot be nullptr!");
  }
  if (serialization_support) {
    if (!match_serialization_support_(*serialization_support, *rosidl_dynamic_data)) {
      throw std::runtime_error(
              "serialization support library identifier does not match dynamic data's!");
    }
  }

  rosidl_dynamic_data_.reset(
    rosidl_dynamic_data,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data)->void {
      rosidl_dynamic_typesupport_dynamic_data_fini(rosidl_dynamic_data);
      free(rosidl_dynamic_data);
    });
}


DynamicData::DynamicData(
  DynamicSerializationSupport::SharedPtr serialization_support,
  std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t> rosidl_dynamic_data)
: serialization_support_(serialization_support),
  rosidl_dynamic_data_(rosidl_dynamic_data),
  is_loaned_(false),
  parent_data_(nullptr)
{
  if (!rosidl_dynamic_data) {
    throw std::runtime_error("rosidl dynamic data cannot be nullptr!");
  }
  if (serialization_support) {
    if (!match_serialization_support_(*serialization_support, *rosidl_dynamic_data)) {
      throw std::runtime_error(
              "serialization support library identifier does not match dynamic data's!");
    }
  }
}


DynamicData::DynamicData(
  DynamicData::SharedPtr parent_data,
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_loaned_data)
: DynamicData(parent_data->get_shared_dynamic_serialization_support(), rosidl_loaned_data)
{
  if (!parent_data) {
    throw std::runtime_error("parent dynamic data cannot be nullptr!");
  }
  if (!rosidl_loaned_data) {
    throw std::runtime_error("loaned rosidl dynamic data cannot be nullptr!");
  }

  parent_data_ = parent_data;
  is_loaned_ = true;
}


DynamicData::DynamicData(const DynamicData & other)
: enable_shared_from_this(),
  serialization_support_(nullptr),
  rosidl_dynamic_data_(nullptr),
  is_loaned_(false),
  parent_data_(nullptr)
{
  DynamicData out = other.clone();
  // We don't copy is_loaned_ or parent_data_ because it's a fresh copy now
  std::swap(serialization_support_, out.serialization_support_);
  std::swap(rosidl_dynamic_data_, out.rosidl_dynamic_data_);
}


DynamicData::DynamicData(DynamicData && other) noexcept
: serialization_support_(std::exchange(other.serialization_support_, nullptr)),
  rosidl_dynamic_data_(std::exchange(other.rosidl_dynamic_data_, nullptr)),
  is_loaned_(other.is_loaned_),
  parent_data_(std::exchange(other.parent_data_, nullptr))
{}


DynamicData &
DynamicData::operator=(const DynamicData & other)
{
  return *this = DynamicData(other);
}


DynamicData &
DynamicData::operator=(DynamicData && other) noexcept
{
  std::swap(serialization_support_, other.serialization_support_);
  std::swap(rosidl_dynamic_data_, other.rosidl_dynamic_data_);
  is_loaned_ = other.is_loaned_;
  std::swap(parent_data_, other.parent_data_);
  return *this;
}


DynamicData::~DynamicData()
{
  if (is_loaned_) {
    if (!parent_data_) {
      RCUTILS_LOG_ERROR("dynamic data is loaned, but parent is missing!!");
    } else {
      rosidl_dynamic_typesupport_dynamic_data_return_loaned_value(
        parent_data_->get_rosidl_dynamic_data(), get_rosidl_dynamic_data());
    }
  }
}


bool
DynamicData::match_serialization_support_(
  const DynamicSerializationSupport & serialization_support,
  const rosidl_dynamic_typesupport_dynamic_data_t & rosidl_dynamic_type_data)
{
  bool out = true;

  if (serialization_support.get_library_identifier() != std::string(
      rosidl_dynamic_type_data.serialization_support->library_identifier))
  {
    RCUTILS_LOG_ERROR("serialization support library identifier does not match dynamic data's");
    out = false;
  }

  // TODO(methylDragon): Can I do this?? Is it portable?
  if (serialization_support.get_rosidl_serialization_support() !=
    rosidl_dynamic_type_data.serialization_support)
  {
    RCUTILS_LOG_ERROR("serialization support pointer does not match dynamic data's");
    out = false;
  }

  return out;
}


// GETTERS =======================================================================================
const std::string
DynamicData::get_library_identifier() const
{
  return std::string(rosidl_dynamic_data_->serialization_support->library_identifier);
}


const std::string
DynamicData::get_name() const
{
  size_t buf_length;
  const char * buf = rosidl_dynamic_typesupport_dynamic_data_get_name(
    get_rosidl_dynamic_data(), &buf_length);
  return std::string(buf, buf_length);
}


rosidl_dynamic_typesupport_dynamic_data_t *
DynamicData::get_rosidl_dynamic_data()
{
  return rosidl_dynamic_data_.get();
}


const rosidl_dynamic_typesupport_dynamic_data_t *
DynamicData::get_rosidl_dynamic_data() const
{
  return rosidl_dynamic_data_.get();
}


std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>
DynamicData::get_shared_rosidl_dynamic_data()
{
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>(
    shared_from_this(), rosidl_dynamic_data_.get());
}


std::shared_ptr<const rosidl_dynamic_typesupport_dynamic_data_t>
DynamicData::get_shared_rosidl_dynamic_data() const
{
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>(
    shared_from_this(), rosidl_dynamic_data_.get());
}


DynamicSerializationSupport::SharedPtr
DynamicData::get_shared_dynamic_serialization_support()
{
  return serialization_support_;
}


DynamicSerializationSupport::ConstSharedPtr
DynamicData::get_shared_dynamic_serialization_support() const
{
  return serialization_support_;
}


size_t
DynamicData::get_item_count() const
{
  return rosidl_dynamic_typesupport_dynamic_data_get_item_count(get_rosidl_dynamic_data());
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::get_member_id(size_t index) const
{
  return rosidl_dynamic_typesupport_dynamic_data_get_member_id_at_index(
    get_rosidl_dynamic_data(), index);
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::get_member_id(const std::string & name) const
{
  return rosidl_dynamic_typesupport_dynamic_data_get_member_id_by_name(
    get_rosidl_dynamic_data(), name.c_str(), name.size());
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::get_array_index(size_t index) const
{
  return rosidl_dynamic_typesupport_dynamic_data_get_array_index(
    get_rosidl_dynamic_data(), index);
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::get_array_index(const std::string & name) const
{
  return get_array_index(get_member_id(name));
}


// METHODS =======================================================================================
DynamicData
DynamicData::clone() const
{
  return DynamicData(
    serialization_support_,
    rosidl_dynamic_typesupport_dynamic_data_clone(get_rosidl_dynamic_data()));
}


DynamicData::SharedPtr
DynamicData::clone_shared() const
{
  return DynamicData::make_shared(
    serialization_support_,
    rosidl_dynamic_typesupport_dynamic_data_clone(get_rosidl_dynamic_data()));
}


bool
DynamicData::equals(const DynamicData & other) const
{
  if (get_library_identifier() != other.get_library_identifier()) {
    throw std::runtime_error("library identifiers don't match");
  }
  return rosidl_dynamic_typesupport_dynamic_data_equals(
    get_rosidl_dynamic_data(), other.get_rosidl_dynamic_data());
}


DynamicData::SharedPtr
DynamicData::loan_value(rosidl_dynamic_typesupport_member_id_t id)
{
  return DynamicData::make_shared(
    shared_from_this(),
    rosidl_dynamic_typesupport_dynamic_data_loan_value(
      get_rosidl_dynamic_data(), id));
}


DynamicData::SharedPtr
DynamicData::loan_value(const std::string & name)
{
  return loan_value(get_member_id(name));
}


void
DynamicData::clear_all_values()
{
  rosidl_dynamic_typesupport_dynamic_data_clear_all_values(get_rosidl_dynamic_data());
}


void
DynamicData::clear_nonkey_values()
{
  rosidl_dynamic_typesupport_dynamic_data_clear_nonkey_values(get_rosidl_dynamic_data());
}


void
DynamicData::clear_value(rosidl_dynamic_typesupport_member_id_t id)
{
  rosidl_dynamic_typesupport_dynamic_data_clear_value(get_rosidl_dynamic_data(), id);
}


void
DynamicData::clear_value(const std::string & name)
{
  clear_value(get_member_id(name));
}


void
DynamicData::clear_sequence()
{
  rosidl_dynamic_typesupport_dynamic_data_clear_sequence_data(get_rosidl_dynamic_data());
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_sequence_data()
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_sequence_data(get_rosidl_dynamic_data(), &out);
  return out;
}


void
DynamicData::remove_sequence_data(rosidl_dynamic_typesupport_member_id_t index)
{
  rosidl_dynamic_typesupport_dynamic_data_remove_sequence_data(
    get_rosidl_dynamic_data(), index);
}


void
DynamicData::print() const
{
  rosidl_dynamic_typesupport_dynamic_data_print(get_rosidl_dynamic_data());
}


bool
DynamicData::serialize(std::shared_ptr<rcutils_uint8_array_t> buffer)
{
  return rosidl_dynamic_typesupport_dynamic_data_serialize(
    get_rosidl_dynamic_data(), buffer.get());
}


bool
DynamicData::deserialize(std::shared_ptr<rcutils_uint8_array_t> buffer)
{
  return rosidl_dynamic_typesupport_dynamic_data_deserialize(
    get_rosidl_dynamic_data(), buffer.get());
}


// MEMBER ACCESS ===================================================================================
// Defined in "detail/dynamic_data_impl.hpp"


// BOUNDED STRING MEMBER ACCESS ====================================================================
const std::string
DynamicData::get_bounded_string_value(
  rosidl_dynamic_typesupport_member_id_t id, size_t string_bound)
{
  size_t buf_length;
  char * buf = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_bounded_string_value(
    get_rosidl_dynamic_data(), id, &buf, &buf_length, string_bound);
  auto out = std::string(buf, buf_length);
  free(buf);
  return out;
}


const std::string
DynamicData::get_bounded_string_value(const std::string & name, size_t string_bound)
{
  return get_bounded_string_value(get_member_id(name), string_bound);
}


const std::u16string
DynamicData::get_bounded_wstring_value(
  rosidl_dynamic_typesupport_member_id_t id, size_t wstring_bound)
{
  size_t buf_length;
  char16_t * buf = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_bounded_wstring_value(
    get_rosidl_dynamic_data(), id, &buf, &buf_length, wstring_bound);
  auto out = std::u16string(buf, buf_length);
  free(buf);
  return out;
}


const std::u16string
DynamicData::get_bounded_wstring_value(const std::string & name, size_t wstring_bound)
{
  return get_bounded_wstring_value(get_member_id(name), wstring_bound);
}


void
DynamicData::set_bounded_string_value(
  rosidl_dynamic_typesupport_member_id_t id, const std::string value, size_t string_bound)
{
  rosidl_dynamic_typesupport_dynamic_data_set_bounded_string_value(
    get_rosidl_dynamic_data(), id, value.c_str(), value.size(), string_bound);
}


void
DynamicData::set_bounded_string_value(
  const std::string & name, const std::string value, size_t string_bound)
{
  set_bounded_string_value(get_member_id(name), value, string_bound);
}


void
DynamicData::set_bounded_wstring_value(
  rosidl_dynamic_typesupport_member_id_t id, const std::u16string value, size_t wstring_bound)
{
  rosidl_dynamic_typesupport_dynamic_data_set_bounded_wstring_value(
    get_rosidl_dynamic_data(), id, value.c_str(), value.size(), wstring_bound);
}


void
DynamicData::set_bounded_wstring_value(
  const std::string & name, const std::u16string value, size_t wstring_bound)
{
  set_bounded_wstring_value(get_member_id(name), value, wstring_bound);
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_bounded_string_value(const std::string value, size_t string_bound)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_bounded_string_value(
    get_rosidl_dynamic_data(), value.c_str(), value.size(), string_bound, &out);
  return out;
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_bounded_wstring_value(const std::u16string value, size_t wstring_bound)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_bounded_wstring_value(
    get_rosidl_dynamic_data(), value.c_str(), value.size(), wstring_bound, &out);
  return out;
}


// NESTED MEMBER ACCESS ============================================================================
DynamicData
DynamicData::get_complex_value(rosidl_dynamic_typesupport_member_id_t id)
{
  rosidl_dynamic_typesupport_dynamic_data_t * out_ptr = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_complex_value(
    get_rosidl_dynamic_data(), id, &out_ptr);
  return DynamicData(get_shared_dynamic_serialization_support(), out_ptr);
}


DynamicData
DynamicData::get_complex_value(const std::string & name)
{
  return get_complex_value(get_member_id(name));
}


DynamicData::SharedPtr
DynamicData::get_complex_value_shared(rosidl_dynamic_typesupport_member_id_t id)
{
  rosidl_dynamic_typesupport_dynamic_data_t * out_ptr = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_complex_value(
    get_rosidl_dynamic_data(), id, &out_ptr);
  return DynamicData::make_shared(get_shared_dynamic_serialization_support(), out_ptr);
}


DynamicData::SharedPtr
DynamicData::get_complex_value_shared(const std::string & name)
{
  return get_complex_value_shared(get_member_id(name));
}


void
DynamicData::set_complex_value(
  rosidl_dynamic_typesupport_member_id_t id, DynamicData & value)
{
  rosidl_dynamic_typesupport_dynamic_data_set_complex_value(
    get_rosidl_dynamic_data(), id, value.get_rosidl_dynamic_data());
}


void
DynamicData::set_complex_value(const std::string & name, DynamicData & value)
{
  set_complex_value(get_member_id(name), value);
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_complex_value_copy(const DynamicData & value)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_complex_value_copy(
    get_rosidl_dynamic_data(), value.get_rosidl_dynamic_data(), &out);
  return out;
}


rosidl_dynamic_typesupport_member_id_t
DynamicData::insert_complex_value(DynamicData & value)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_complex_value(
    get_rosidl_dynamic_data(), value.get_rosidl_dynamic_data(), &out);
  return out;
}
