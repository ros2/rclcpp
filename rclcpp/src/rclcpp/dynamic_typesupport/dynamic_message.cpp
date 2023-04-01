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

#include <rosidl_dynamic_typesupport/api/dynamic_type.h>
#include <rosidl_dynamic_typesupport/api/dynamic_data.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <memory>
#include <string>

#include "rcl/types.h"
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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DETAIL__DYNAMIC_MESSAGE_IMPL_HPP_
// Template specialization implementations
#include "rclcpp/dynamic_typesupport/detail/dynamic_message_impl.hpp"
#endif


// CONSTRUCTION ==================================================================================
DynamicMessage::DynamicMessage(const DynamicMessageTypeBuilder::SharedPtr dynamic_type_builder)
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
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type_builder(
    rosidl_dynamic_type_builder, &rosidl_dynamic_data);
  if (ret != RCUTILS_RET_OK || !rosidl_dynamic_data) {
    throw std::runtime_error("could not create new dynamic data object from dynamic type builder");
  }

  rosidl_dynamic_data_.reset(
    rosidl_dynamic_data,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data)->void {
      rosidl_dynamic_typesupport_dynamic_data_fini(rosidl_dynamic_data);
      delete rosidl_dynamic_data;
    });
}


DynamicMessage::DynamicMessage(const DynamicMessageType::SharedPtr dynamic_type)
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
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type(
    rosidl_dynamic_type, &rosidl_dynamic_data);
  if (ret != RCUTILS_RET_OK || !rosidl_dynamic_data) {
    throw std::runtime_error(
            std::string("could not create new dynamic data object from dynamic type") +
            rcl_get_error_string().str);
  }

  rosidl_dynamic_data_.reset(
    rosidl_dynamic_data,
    // Custom deleter
    [](rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data)->void {
      rosidl_dynamic_typesupport_dynamic_data_fini(rosidl_dynamic_data);
      delete rosidl_dynamic_data;
    });
}


DynamicMessage::DynamicMessage(
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
      delete rosidl_dynamic_data;
    });
}


DynamicMessage::DynamicMessage(
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


DynamicMessage::DynamicMessage(
  DynamicMessage::SharedPtr parent_data,
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_loaned_data)
: serialization_support_(parent_data->get_shared_dynamic_serialization_support()),
  rosidl_dynamic_data_(nullptr),
  is_loaned_(true),
  parent_data_(nullptr)
{
  if (!parent_data) {
    throw std::runtime_error("parent dynamic data cannot be nullptr!");
  }
  if (!rosidl_loaned_data) {
    throw std::runtime_error("loaned rosidl dynamic data cannot be nullptr!");
  }

  if (serialization_support_) {
    if (!match_serialization_support_(*serialization_support_, *rosidl_loaned_data)) {
      throw std::runtime_error(
              "serialization support library identifier does not match loaned dynamic data's!");
    }
  }

  rosidl_dynamic_data_.reset(
    rosidl_loaned_data,
    // Custom no-op deleter
    [](rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data)->void {
      // Data fini and destruction is deferred to return_loaned_value()
      (void) rosidl_dynamic_data;
    });
  parent_data_ = parent_data;
}


DynamicMessage::DynamicMessage(const DynamicMessage & other)
: enable_shared_from_this(),
  serialization_support_(nullptr),
  rosidl_dynamic_data_(nullptr),
  is_loaned_(false),
  parent_data_(nullptr)
{
  DynamicMessage out = other.clone();
  // We don't copy is_loaned_ or parent_data_ because it's a fresh copy now
  std::swap(serialization_support_, out.serialization_support_);
  std::swap(rosidl_dynamic_data_, out.rosidl_dynamic_data_);
}


DynamicMessage::DynamicMessage(DynamicMessage && other) noexcept
: serialization_support_(std::exchange(other.serialization_support_, nullptr)),
  rosidl_dynamic_data_(std::exchange(other.rosidl_dynamic_data_, nullptr)),
  is_loaned_(other.is_loaned_),
  parent_data_(std::exchange(other.parent_data_, nullptr))
{}


DynamicMessage &
DynamicMessage::operator=(const DynamicMessage & other)
{
  return *this = DynamicMessage(other);
}


DynamicMessage &
DynamicMessage::operator=(DynamicMessage && other) noexcept
{
  std::swap(serialization_support_, other.serialization_support_);
  std::swap(rosidl_dynamic_data_, other.rosidl_dynamic_data_);
  is_loaned_ = other.is_loaned_;
  std::swap(parent_data_, other.parent_data_);
  return *this;
}


DynamicMessage::~DynamicMessage()
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
DynamicMessage::match_serialization_support_(
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
DynamicMessage::get_library_identifier() const
{
  return std::string(rosidl_dynamic_data_->serialization_support->library_identifier);
}


const std::string
DynamicMessage::get_name() const
{
  size_t buf_length;
  const char * buf;
  if (
    rosidl_dynamic_typesupport_dynamic_data_get_name(
      get_rosidl_dynamic_data(), &buf,
      &buf_length) !=
    RCUTILS_RET_OK)
  {
    throw std::runtime_error(
            std::string("could not get name for dynamic data") + rcl_get_error_string().str);
  }
  return std::string(buf, buf_length);
}


rosidl_dynamic_typesupport_dynamic_data_t *
DynamicMessage::get_rosidl_dynamic_data()
{
  return rosidl_dynamic_data_.get();
}


const rosidl_dynamic_typesupport_dynamic_data_t *
DynamicMessage::get_rosidl_dynamic_data() const
{
  return rosidl_dynamic_data_.get();
}


std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>
DynamicMessage::get_shared_rosidl_dynamic_data()
{
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>(
    shared_from_this(), rosidl_dynamic_data_.get());
}


std::shared_ptr<const rosidl_dynamic_typesupport_dynamic_data_t>
DynamicMessage::get_shared_rosidl_dynamic_data() const
{
  return std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t>(
    shared_from_this(), rosidl_dynamic_data_.get());
}


DynamicSerializationSupport::SharedPtr
DynamicMessage::get_shared_dynamic_serialization_support()
{
  return serialization_support_;
}


DynamicSerializationSupport::ConstSharedPtr
DynamicMessage::get_shared_dynamic_serialization_support() const
{
  return serialization_support_;
}


size_t
DynamicMessage::get_item_count() const
{
  size_t item_count;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_get_item_count(
    get_rosidl_dynamic_data(), &item_count);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not get item count of dynamic data");
  }
  return item_count;
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::get_member_id(size_t index) const
{
  rosidl_dynamic_typesupport_member_id_t member_id;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_get_member_id_at_index(
    get_rosidl_dynamic_data(), index, &member_id);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not member id of dynamic data element by index");
  }
  return member_id;
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::get_member_id(const std::string & name) const
{
  rosidl_dynamic_typesupport_member_id_t member_id;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_get_member_id_by_name(
    get_rosidl_dynamic_data(), name.c_str(), name.size(), &member_id);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not member id of dynamic data element by name");
  }
  return member_id;
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::get_array_index(size_t index) const
{
  rosidl_dynamic_typesupport_member_id_t array_index;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_get_array_index(
    get_rosidl_dynamic_data(), index, &array_index);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not array index of dynamic data element by index");
  }
  return array_index;
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::get_array_index(const std::string & name) const
{
  return get_array_index(get_member_id(name));
}


// METHODS =======================================================================================
DynamicMessage
DynamicMessage::clone() const
{
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data = nullptr;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_clone(
    get_rosidl_dynamic_data(), &rosidl_dynamic_data);
  if (ret != RCUTILS_RET_OK || !rosidl_dynamic_data) {
    throw std::runtime_error(
            std::string("could not clone dynamic data: ") + rcl_get_error_string().str);
  }
  return DynamicMessage(serialization_support_, rosidl_dynamic_data);
}


DynamicMessage::SharedPtr
DynamicMessage::clone_shared() const
{
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data = nullptr;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_clone(
    get_rosidl_dynamic_data(), &rosidl_dynamic_data);
  if (ret != RCUTILS_RET_OK || !rosidl_dynamic_data) {
    throw std::runtime_error(
            std::string("could not clone dynamic data: ") + rcl_get_error_string().str);
  }
  return DynamicMessage::make_shared(serialization_support_, rosidl_dynamic_data);
}


DynamicMessage
DynamicMessage::init_from_type(DynamicMessageType & type) const
{
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data = nullptr;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type(
    type.get_rosidl_dynamic_type(), &rosidl_dynamic_data);
  if (ret != RCUTILS_RET_OK || !rosidl_dynamic_data) {
    throw std::runtime_error("could not create new dynamic data object from dynamic type");
  }
  return DynamicMessage(serialization_support_, rosidl_dynamic_data);
}


DynamicMessage::SharedPtr
DynamicMessage::init_from_type_shared(DynamicMessageType & type) const
{
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data = nullptr;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type(
    type.get_rosidl_dynamic_type(), &rosidl_dynamic_data);
  if (ret != RCUTILS_RET_OK || !rosidl_dynamic_data) {
    throw std::runtime_error("could not create new dynamic data object from dynamic type");
  }
  return DynamicMessage::make_shared(serialization_support_, rosidl_dynamic_data);
}


bool
DynamicMessage::equals(const DynamicMessage & other) const
{
  if (get_library_identifier() != other.get_library_identifier()) {
    throw std::runtime_error("library identifiers don't match");
  }
  bool equals;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_equals(
    get_rosidl_dynamic_data(), other.get_rosidl_dynamic_data(), &equals);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("could not equate dynamic messages");
  }
  return equals;
}


DynamicMessage::SharedPtr
DynamicMessage::loan_value(rosidl_dynamic_typesupport_member_id_t id)
{
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data = nullptr;
  rcutils_ret_t ret = rosidl_dynamic_typesupport_dynamic_data_loan_value(
    get_rosidl_dynamic_data(), id, &rosidl_dynamic_data);
  if (ret != RCUTILS_RET_OK || !rosidl_dynamic_data) {
    throw std::runtime_error(
            std::string("could not loan dynamic data: ") + rcl_get_error_string().str);
  }
  return DynamicMessage::make_shared(shared_from_this(), rosidl_dynamic_data);
}


DynamicMessage::SharedPtr
DynamicMessage::loan_value(const std::string & name)
{
  return loan_value(get_member_id(name));
}


void
DynamicMessage::clear_all_values()
{
  rosidl_dynamic_typesupport_dynamic_data_clear_all_values(get_rosidl_dynamic_data());
}


void
DynamicMessage::clear_nonkey_values()
{
  rosidl_dynamic_typesupport_dynamic_data_clear_nonkey_values(get_rosidl_dynamic_data());
}


void
DynamicMessage::clear_value(rosidl_dynamic_typesupport_member_id_t id)
{
  rosidl_dynamic_typesupport_dynamic_data_clear_value(get_rosidl_dynamic_data(), id);
}


void
DynamicMessage::clear_value(const std::string & name)
{
  clear_value(get_member_id(name));
}


void
DynamicMessage::clear_sequence()
{
  rosidl_dynamic_typesupport_dynamic_data_clear_sequence_data(get_rosidl_dynamic_data());
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::insert_sequence_data()
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_sequence_data(get_rosidl_dynamic_data(), &out);
  return out;
}


void
DynamicMessage::remove_sequence_data(rosidl_dynamic_typesupport_member_id_t index)
{
  rosidl_dynamic_typesupport_dynamic_data_remove_sequence_data(
    get_rosidl_dynamic_data(), index);
}


void
DynamicMessage::print() const
{
  rosidl_dynamic_typesupport_dynamic_data_print(get_rosidl_dynamic_data());
}


bool
DynamicMessage::serialize(rcl_serialized_message_t * buffer)
{
  bool success;
  rcutils_ret_t ret =
    rosidl_dynamic_typesupport_dynamic_data_serialize(get_rosidl_dynamic_data(), buffer, &success);
  if (ret != RCUTILS_RET_OK || !success) {
    throw std::runtime_error(
            std::string("could serialize loan dynamic data: ") + rcl_get_error_string().str);
  }
  return success;
}


bool
DynamicMessage::deserialize(rcl_serialized_message_t * buffer)
{
  bool success;
  rcutils_ret_t ret =
    rosidl_dynamic_typesupport_dynamic_data_deserialize(
    get_rosidl_dynamic_data(), buffer,
    &success);
  if (ret != RCUTILS_RET_OK || !success) {
    throw std::runtime_error(
            std::string("could deserialize loan dynamic data: ") + rcl_get_error_string().str);
  }
  return success;
}


// MEMBER ACCESS ===================================================================================
// Defined in "detail/dynamic_message_impl.hpp"


// FIXED STRING MEMBER ACCESS ======================================================================
const std::string
DynamicMessage::get_fixed_string_value(
  rosidl_dynamic_typesupport_member_id_t id, size_t string_length)
{
  size_t buf_length;
  char * buf = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_fixed_string_value(
    get_rosidl_dynamic_data(), id, &buf, &buf_length, string_length);
  auto out = std::string(buf, buf_length);
  delete buf;
  return out;
}


const std::string
DynamicMessage::get_fixed_string_value(const std::string & name, size_t string_length)
{
  return get_fixed_string_value(get_member_id(name), string_length);
}


const std::u16string
DynamicMessage::get_fixed_wstring_value(
  rosidl_dynamic_typesupport_member_id_t id, size_t wstring_length)
{
  size_t buf_length;
  char16_t * buf = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_fixed_wstring_value(
    get_rosidl_dynamic_data(), id, &buf, &buf_length, wstring_length);
  auto out = std::u16string(buf, buf_length);
  delete buf;
  return out;
}


const std::u16string
DynamicMessage::get_fixed_wstring_value(const std::string & name, size_t wstring_length)
{
  return get_fixed_wstring_value(get_member_id(name), wstring_length);
}


void
DynamicMessage::set_fixed_string_value(
  rosidl_dynamic_typesupport_member_id_t id, const std::string value, size_t string_length)
{
  rosidl_dynamic_typesupport_dynamic_data_set_fixed_string_value(
    get_rosidl_dynamic_data(), id, value.c_str(), value.size(), string_length);
}


void
DynamicMessage::set_fixed_string_value(
  const std::string & name, const std::string value, size_t string_length)
{
  set_fixed_string_value(get_member_id(name), value, string_length);
}


void
DynamicMessage::set_fixed_wstring_value(
  rosidl_dynamic_typesupport_member_id_t id, const std::u16string value, size_t wstring_length)
{
  rosidl_dynamic_typesupport_dynamic_data_set_fixed_wstring_value(
    get_rosidl_dynamic_data(), id, value.c_str(), value.size(), wstring_length);
}


void
DynamicMessage::set_fixed_wstring_value(
  const std::string & name, const std::u16string value, size_t wstring_length)
{
  set_fixed_wstring_value(get_member_id(name), value, wstring_length);
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::insert_fixed_string_value(const std::string value, size_t string_length)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_fixed_string_value(
    get_rosidl_dynamic_data(), value.c_str(), value.size(), string_length, &out);
  return out;
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::insert_fixed_wstring_value(const std::u16string value, size_t wstring_length)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_fixed_wstring_value(
    get_rosidl_dynamic_data(), value.c_str(), value.size(), wstring_length, &out);
  return out;
}


// BOUNDED STRING MEMBER ACCESS ====================================================================
const std::string
DynamicMessage::get_bounded_string_value(
  rosidl_dynamic_typesupport_member_id_t id, size_t string_bound)
{
  size_t buf_length;
  char * buf = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_bounded_string_value(
    get_rosidl_dynamic_data(), id, &buf, &buf_length, string_bound);
  auto out = std::string(buf, buf_length);
  delete buf;
  return out;
}


const std::string
DynamicMessage::get_bounded_string_value(const std::string & name, size_t string_bound)
{
  return get_bounded_string_value(get_member_id(name), string_bound);
}


const std::u16string
DynamicMessage::get_bounded_wstring_value(
  rosidl_dynamic_typesupport_member_id_t id, size_t wstring_bound)
{
  size_t buf_length;
  char16_t * buf = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_bounded_wstring_value(
    get_rosidl_dynamic_data(), id, &buf, &buf_length, wstring_bound);
  auto out = std::u16string(buf, buf_length);
  delete buf;
  return out;
}


const std::u16string
DynamicMessage::get_bounded_wstring_value(const std::string & name, size_t wstring_bound)
{
  return get_bounded_wstring_value(get_member_id(name), wstring_bound);
}


void
DynamicMessage::set_bounded_string_value(
  rosidl_dynamic_typesupport_member_id_t id, const std::string value, size_t string_bound)
{
  rosidl_dynamic_typesupport_dynamic_data_set_bounded_string_value(
    get_rosidl_dynamic_data(), id, value.c_str(), value.size(), string_bound);
}


void
DynamicMessage::set_bounded_string_value(
  const std::string & name, const std::string value, size_t string_bound)
{
  set_bounded_string_value(get_member_id(name), value, string_bound);
}


void
DynamicMessage::set_bounded_wstring_value(
  rosidl_dynamic_typesupport_member_id_t id, const std::u16string value, size_t wstring_bound)
{
  rosidl_dynamic_typesupport_dynamic_data_set_bounded_wstring_value(
    get_rosidl_dynamic_data(), id, value.c_str(), value.size(), wstring_bound);
}


void
DynamicMessage::set_bounded_wstring_value(
  const std::string & name, const std::u16string value, size_t wstring_bound)
{
  set_bounded_wstring_value(get_member_id(name), value, wstring_bound);
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::insert_bounded_string_value(const std::string value, size_t string_bound)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_bounded_string_value(
    get_rosidl_dynamic_data(), value.c_str(), value.size(), string_bound, &out);
  return out;
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::insert_bounded_wstring_value(const std::u16string value, size_t wstring_bound)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_bounded_wstring_value(
    get_rosidl_dynamic_data(), value.c_str(), value.size(), wstring_bound, &out);
  return out;
}


// NESTED MEMBER ACCESS ============================================================================
DynamicMessage
DynamicMessage::get_complex_value(rosidl_dynamic_typesupport_member_id_t id)
{
  rosidl_dynamic_typesupport_dynamic_data_t * out_ptr = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_complex_value(
    get_rosidl_dynamic_data(), id, &out_ptr);
  return DynamicMessage(get_shared_dynamic_serialization_support(), out_ptr);
}


DynamicMessage
DynamicMessage::get_complex_value(const std::string & name)
{
  return get_complex_value(get_member_id(name));
}


DynamicMessage::SharedPtr
DynamicMessage::get_complex_value_shared(rosidl_dynamic_typesupport_member_id_t id)
{
  rosidl_dynamic_typesupport_dynamic_data_t * out_ptr = nullptr;
  rosidl_dynamic_typesupport_dynamic_data_get_complex_value(
    get_rosidl_dynamic_data(), id, &out_ptr);
  return DynamicMessage::make_shared(get_shared_dynamic_serialization_support(), out_ptr);
}


DynamicMessage::SharedPtr
DynamicMessage::get_complex_value_shared(const std::string & name)
{
  return get_complex_value_shared(get_member_id(name));
}


void
DynamicMessage::set_complex_value(
  rosidl_dynamic_typesupport_member_id_t id, DynamicMessage & value)
{
  rosidl_dynamic_typesupport_dynamic_data_set_complex_value(
    get_rosidl_dynamic_data(), id, value.get_rosidl_dynamic_data());
}


void
DynamicMessage::set_complex_value(const std::string & name, DynamicMessage & value)
{
  set_complex_value(get_member_id(name), value);
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::insert_complex_value_copy(const DynamicMessage & value)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_complex_value_copy(
    get_rosidl_dynamic_data(), value.get_rosidl_dynamic_data(), &out);
  return out;
}


rosidl_dynamic_typesupport_member_id_t
DynamicMessage::insert_complex_value(DynamicMessage & value)
{
  rosidl_dynamic_typesupport_member_id_t out;
  rosidl_dynamic_typesupport_dynamic_data_insert_complex_value(
    get_rosidl_dynamic_data(), value.get_rosidl_dynamic_data(), &out);
  return out;
}
