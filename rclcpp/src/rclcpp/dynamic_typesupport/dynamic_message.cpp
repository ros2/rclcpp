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

#include "rclcpp/dynamic_typesupport/dynamic_message.hpp"

#include "rclcpp/dynamic_typesupport/dynamic_data.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_type_builder.hpp"
#include "rclcpp/exceptions.hpp"
#include "rcutils/logging_macros.h"

#include <rosidl_dynamic_typesupport/api/dynamic_type.h>
#include <rosidl_dynamic_typesupport/api/dynamic_data.h>
#include <rosidl_dynamic_typesupport/types.h>

using rclcpp::dynamic_typesupport::DynamicMessage;
using rclcpp::dynamic_typesupport::DynamicSerializationSupport;
using rclcpp::dynamic_typesupport::DynamicType;
using rclcpp::dynamic_typesupport::DynamicTypeBuilder;


// CONSTRUCTION ==================================================================================
DynamicMessage::DynamicMessage(const DynamicTypeBuilder::SharedPtr dynamic_type_builder)
: DynamicData(dynamic_type_builder) {}


DynamicMessage::DynamicMessage(const DynamicType::SharedPtr dynamic_type)
: DynamicData(dynamic_type) {}


DynamicMessage::DynamicMessage(
  DynamicSerializationSupport::SharedPtr serialization_support,
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_dynamic_data)
: DynamicData(serialization_support, rosidl_dynamic_data) {}


DynamicMessage::DynamicMessage(
  DynamicSerializationSupport::SharedPtr serialization_support,
  std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t> rosidl_dynamic_data)
: DynamicData(serialization_support, rosidl_dynamic_data) {}


DynamicMessage::DynamicMessage(
  DynamicData::SharedPtr parent_data,
  rosidl_dynamic_typesupport_dynamic_data_t * rosidl_loaned_data)
: DynamicData(parent_data, rosidl_loaned_data) {}
