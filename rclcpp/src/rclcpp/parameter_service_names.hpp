// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__PARAMETER_SERVICE_NAMES_HPP_
#define RCLCPP__PARAMETER_SERVICE_NAMES_HPP_

namespace rclcpp
{
namespace parameter_service_names
{

static constexpr const char * get_parameters = "get_parameters";
static constexpr const char * get_parameter_types = "get_parameter_types";
static constexpr const char * set_parameters = "set_parameters";
static constexpr const char * set_parameters_atomically = "set_parameters_atomically";
static constexpr const char * describe_parameters = "describe_parameters";
static constexpr const char * list_parameters = "list_parameters";

}  // namespace parameter_service_names
}  // namespace rclcpp

#endif  // RCLCPP__PARAMETER_SERVICE_NAMES_HPP_
