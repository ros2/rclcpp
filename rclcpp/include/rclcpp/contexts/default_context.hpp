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

#ifndef RCLCPP__CONTEXTS__DEFAULT_CONTEXT_HPP_
#define RCLCPP__CONTEXTS__DEFAULT_CONTEXT_HPP_

#include "rclcpp/context.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace contexts
{

class DefaultContext : public rclcpp::Context
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DefaultContext)

  RCLCPP_PUBLIC
  DefaultContext();
};

RCLCPP_PUBLIC
DefaultContext::SharedPtr
get_global_default_context();

}  // namespace contexts
}  // namespace rclcpp

#endif  // RCLCPP__CONTEXTS__DEFAULT_CONTEXT_HPP_
