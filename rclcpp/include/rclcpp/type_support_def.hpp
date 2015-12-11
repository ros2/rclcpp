// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TYPE_SUPPORT_DEF_HPP_
#define RCLCPP__TYPE_SUPPORT_DEF_HPP_

#include "rclcpp/type_support_decl.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"
#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/srv/get_parameter_types.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rosidl_generator_cpp/message_type_support.hpp"
#include "rosidl_generator_cpp/service_type_support.hpp"

inline const rosidl_message_type_support_t *
rclcpp::type_support::get_intra_process_message_msg_type_support()
{
  return rosidl_generator_cpp::get_message_type_support_handle<
    rcl_interfaces::msg::IntraProcessMessage
  >();
}

inline const rosidl_message_type_support_t *
rclcpp::type_support::get_parameter_event_msg_type_support()
{
  return rosidl_generator_cpp::get_message_type_support_handle<
    rcl_interfaces::msg::ParameterEvent
  >();
}

inline const rosidl_message_type_support_t *
rclcpp::type_support::get_set_parameters_result_msg_type_support()
{
  return rosidl_generator_cpp::get_message_type_support_handle<
    rcl_interfaces::msg::SetParametersResult
  >();
}

inline const rosidl_message_type_support_t *
rclcpp::type_support::get_parameter_descriptor_msg_type_support()
{
  return rosidl_generator_cpp::get_message_type_support_handle<
    rcl_interfaces::msg::ParameterDescriptor
  >();
}

inline const rosidl_message_type_support_t *
rclcpp::type_support::get_list_parameters_result_msg_type_support()
{
  return rosidl_generator_cpp::get_message_type_support_handle<
    rcl_interfaces::msg::ListParametersResult
  >();
}

inline const rosidl_service_type_support_t *
rclcpp::type_support::get_get_parameters_srv_type_support()
{
  return rosidl_generator_cpp::get_service_type_support_handle<
    rcl_interfaces::srv::GetParameters
  >();
}

inline const rosidl_service_type_support_t *
rclcpp::type_support::get_get_parameter_types_srv_type_support()
{
  return rosidl_generator_cpp::get_service_type_support_handle<
    rcl_interfaces::srv::GetParameterTypes
  >();
}

inline const rosidl_service_type_support_t *
rclcpp::type_support::get_set_parameters_srv_type_support()
{
  return rosidl_generator_cpp::get_service_type_support_handle<
    rcl_interfaces::srv::SetParameters
  >();
}

inline const rosidl_service_type_support_t *
rclcpp::type_support::get_list_parameters_srv_type_support()
{
  return rosidl_generator_cpp::get_service_type_support_handle<
    rcl_interfaces::srv::ListParameters
  >();
}

inline const rosidl_service_type_support_t *
rclcpp::type_support::get_describe_parameters_srv_type_support()
{
  return rosidl_generator_cpp::get_service_type_support_handle<
    rcl_interfaces::srv::DescribeParameters
  >();
}

inline const rosidl_service_type_support_t *
rclcpp::type_support::get_set_parameters_atomically_srv_type_support()
{
  return rosidl_generator_cpp::get_service_type_support_handle<
    rcl_interfaces::srv::SetParametersAtomically
  >();
}

#endif  // RCLCPP__TYPE_SUPPORT_DEF_HPP_
