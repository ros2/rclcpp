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

#ifndef RCLCPP__NODE_INTERFACES__NODE_SERVICE_INTROSPECTION_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_SERVICE_INTROSPECTION_HPP_

#include <vector>
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_service_introspection_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{
class NodeServiceIntrospection : public NodeServiceIntrospectionInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeServiceIntrospection)

  RCLCPP_PUBLIC
  explicit NodeServiceIntrospection(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters);

  RCLCPP_PUBLIC
  size_t
  register_service(rclcpp::ServiceBase::SharedPtr service) override;

  RCLCPP_PUBLIC
  size_t
  register_client(rclcpp::ClientBase::SharedPtr client) override;

  RCLCPP_PUBLIC
  ~NodeServiceIntrospection() override = default;

private:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  std::vector<rclcpp::ServiceBase::WeakPtr> services_;
  std::vector<rclcpp::ClientBase::WeakPtr> clients_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
    post_set_parameters_callback_handle_;
};

}  // namespace node_interfaces
}  // namespace rclcpp
#endif  // RCLCPP__NODE_INTERFACES__NODE_SERVICE_INTROSPECTION_HPP_
