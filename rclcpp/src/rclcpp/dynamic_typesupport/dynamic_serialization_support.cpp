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
: DynamicSerializationSupport::DynamicSerializationSupport("", allocator)
{
  throw std::runtime_error("Unimplemented");
}

DynamicSerializationSupport::DynamicSerializationSupport(
  const std::string & /*serialization_library_name*/,
  rcl_allocator_t /*allocator*/)
: rosidl_serialization_support_(
    rosidl_dynamic_typesupport_get_zero_initialized_serialization_support())
{
  throw std::runtime_error("Unimplemented");
}

DynamicSerializationSupport::~DynamicSerializationSupport()
{}  // STUBBED
