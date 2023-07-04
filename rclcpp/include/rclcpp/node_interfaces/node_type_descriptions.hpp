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

#ifndef RCLCPP__NODE_INTERFACES__NODE_TYPE_DESCRIPTIONS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_TYPE_DESCRIPTIONS_HPP_

#include <memory>

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_type_descriptions_interface.hpp"
#include "rclcpp/visibility_control.hpp"

#include "type_description_interfaces/srv/get_type_description.h"


namespace type_description_interfaces
{
namespace srv
{
// Helper wrapper for rclcpp::Service to access ::Request and ::Response types for allocation.
struct GetTypeDescription__C
{
  using Request = type_description_interfaces__srv__GetTypeDescription_Request;
  using Response = type_description_interfaces__srv__GetTypeDescription_Response;
  using Event = type_description_interfaces__srv__GetTypeDescription_Event;
};
}  // namespace srv
}  // namespace type_description_interfaces

namespace rosidl_typesupport_cpp
{
// Helper function for C typesupport.
template<>
RCLCPP_PUBLIC
rosidl_service_type_support_t const *
get_service_type_support_handle<type_description_interfaces::srv::GetTypeDescription__C>();
}  // namespace rosidl_typesupport_cpp

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeTypeDescriptions part of the Node API.
class NodeTypeDescriptions : public NodeTypeDescriptionsInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeTypeDescriptions)

  RCLCPP_PUBLIC
  explicit NodeTypeDescriptions(
    NodeBaseInterface::SharedPtr node_base,
    NodeLoggingInterface::SharedPtr node_logging,
    NodeParametersInterface::SharedPtr node_parameters,
    NodeServicesInterface::SharedPtr node_services);

  RCLCPP_PUBLIC
  virtual
  ~NodeTypeDescriptions();

private:
  RCLCPP_DISABLE_COPY(NodeTypeDescriptions)

  // Pimpl for future backport ABI stability assistance, not for general functionality
  class NodeTypeDescriptionsImpl;
  using ServiceT = type_description_interfaces::srv::GetTypeDescription__C;

  rclcpp::Logger logger_;
  rclcpp::Service<ServiceT>::SharedPtr type_description_srv_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;

  std::unique_ptr<NodeTypeDescriptionsImpl> impl_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_TYPE_DESCRIPTIONS_HPP_
