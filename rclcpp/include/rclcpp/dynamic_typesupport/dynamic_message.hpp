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

#ifndef RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_
#define RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_


#include <memory>
#include <string>

#include "rclcpp/dynamic_typesupport/dynamic_data.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_serialization_support.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_type.hpp"
#include "rclcpp/dynamic_typesupport/dynamic_type_builder.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#include <rosidl_dynamic_typesupport/types.h>


namespace rclcpp
{
namespace dynamic_typesupport
{


// NOTE(methylDragon): We just alias the type in this case...
//                     I'd have made a wrapper class but then I'd need to redirect every single
//                     method (or dynamic cast everywhere else)
//
/// This type maps to the underlying rosidl_dynamic_typesupport_dynamic_data_t, rather than the equivalent
/// rosidl_dynamic_typesupport_dynamic_data_t struct.
using DynamicMessage = DynamicData;

}  // namespace dynamic_typesupport
}  // namespace rclcpp

#endif  // RCLCPP__DYNAMIC_TYPESUPPORT__DYNAMIC_MESSAGE_HPP_
