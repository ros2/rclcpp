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

#include <rclcpp/detail/resolve_intra_process_buffer_type.hpp>

namespace rclcpp
{

namespace detail
{
rclcpp::IntraProcessBufferType
resolve_intra_process_buffer_type(
  const rclcpp::IntraProcessBufferType buffer_type)
{
  if (buffer_type == IntraProcessBufferType::CallbackDefault) {
    throw std::invalid_argument(
            "IntraProcessBufferType::CallbackDefault is not allowed "
            "when there is no callback function");
  }

  return buffer_type;
}

}  // namespace detail

}  // namespace rclcpp
