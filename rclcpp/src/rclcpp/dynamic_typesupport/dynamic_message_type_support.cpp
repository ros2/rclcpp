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
#include <rosidl_runtime_c/type_description/type_source__functions.h>
#include <rosidl_runtime_c/type_description/type_source__struct.h>

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

DynamicMessageTypeSupport::~DynamicMessageTypeSupport()
{}  // STUBBED
