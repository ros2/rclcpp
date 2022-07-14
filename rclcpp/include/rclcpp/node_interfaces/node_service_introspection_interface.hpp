// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_SERVICE_INTROSPECTION_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_SERVICE_INTROSPECTION_INTERFACE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/client.hpp"

namespace rclcpp
{
namespace node_interfaces
{
class NodeServiceIntrospectionInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeServiceIntrospectionInterface)

  RCLCPP_PUBLIC
  size_t
  virtual register_service(rclcpp::ServiceBase::SharedPtr service) = 0;

  RCLCPP_PUBLIC
  size_t
  virtual register_client(rclcpp::ClientBase::SharedPtr client) = 0;

  RCLCPP_PUBLIC
  virtual
  ~NodeServiceIntrospectionInterface() = default;
};

}  // namespace node_interfaces
}  // namespace rclcpp
#endif  // RCLCPP__NODE_INTERFACES__NODE_SERVICE_INTROSPECTION_INTERFACE_HPP_
