// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_SERVICES_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_SERVICES_HPP_

#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeServices part of the Node API.
class NodeServices : public NodeServicesInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeServices)

  RCLCPP_PUBLIC
  explicit NodeServices(rclcpp::node_interfaces::NodeBaseInterface * node_base);

  RCLCPP_PUBLIC
  virtual
  ~NodeServices();

  RCLCPP_PUBLIC
  void
  add_client(
    rclcpp::ClientBase::SharedPtr client_base_ptr,
    rclcpp::CallbackGroup::SharedPtr group) override;

  RCLCPP_PUBLIC
  void
  add_service(
    rclcpp::ServiceBase::SharedPtr service_base_ptr,
    rclcpp::CallbackGroup::SharedPtr group) override;

  RCLCPP_PUBLIC
  std::string
  resolve_service_name(const std::string & name, bool only_expand = false) const override;

private:
  RCLCPP_DISABLE_COPY(NodeServices)

  rclcpp::node_interfaces::NodeBaseInterface * node_base_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_SERVICES_HPP_
