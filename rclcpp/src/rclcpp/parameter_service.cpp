// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/parameter_service.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "./parameter_service_names.hpp"

using rclcpp::parameter_service::ParameterService;

ParameterService::ParameterService(
  const rclcpp::node::Node::SharedPtr node,
  const rmw_qos_profile_t & qos_profile)
: node_(node)
{
  std::weak_ptr<rclcpp::node::Node> captured_node = node_;
  // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
  get_parameters_service_ = node_->create_service<rcl_interfaces::srv::GetParameters>(
    std::string(node_->get_name()) + "/" + parameter_service_names::get_parameters,
    [captured_node](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response)
    {
      auto node = captured_node.lock();
      if (!node) {
        return;
      }
      auto values = node->get_parameters(request->names);
      for (auto & pvariant : values) {
        response->values.push_back(pvariant.get_parameter_value());
      }
    },
    qos_profile
  );

  get_parameter_types_service_ = node_->create_service<rcl_interfaces::srv::GetParameterTypes>(
    std::string(node_->get_name()) + "/" + parameter_service_names::get_parameter_types,
    [captured_node](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Request> request,
      std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Response> response)
    {
      auto node = captured_node.lock();
      if (!node) {
        return;
      }
      auto types = node->get_parameter_types(request->names);
      std::transform(types.cbegin(), types.cend(),
      std::back_inserter(response->types), [](const uint8_t & type) {
        return static_cast<rclcpp::parameter::ParameterType>(type);
      });
    },
    qos_profile
  );

  set_parameters_service_ = node_->create_service<rcl_interfaces::srv::SetParameters>(
    std::string(node_->get_name()) + "/" + parameter_service_names::set_parameters,
    [captured_node](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response)
    {
      auto node = captured_node.lock();
      if (!node) {
        return;
      }
      std::vector<rclcpp::parameter::ParameterVariant> pvariants;
      for (auto & p : request->parameters) {
        pvariants.push_back(rclcpp::parameter::ParameterVariant::from_parameter(p));
      }
      auto results = node->set_parameters(pvariants);
      response->results = results;
    },
    qos_profile
  );

  set_parameters_atomically_service_ =
    node_->create_service<rcl_interfaces::srv::SetParametersAtomically>(
    std::string(node_->get_name()) + "/" + parameter_service_names::set_parameters_atomically,
    [captured_node](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response)
    {
      auto node = captured_node.lock();
      if (!node) {
        return;
      }
      std::vector<rclcpp::parameter::ParameterVariant> pvariants;
      std::transform(request->parameters.cbegin(), request->parameters.cend(),
      std::back_inserter(pvariants),
      [](const rcl_interfaces::msg::Parameter & p) {
        return rclcpp::parameter::ParameterVariant::
        from_parameter(p);
      });
      auto result = node->set_parameters_atomically(pvariants);
      response->result = result;
    },
    qos_profile
  );

  describe_parameters_service_ = node_->create_service<rcl_interfaces::srv::DescribeParameters>(
    std::string(node_->get_name()) + "/" + parameter_service_names::describe_parameters,
    [captured_node](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Response> response)
    {
      auto node = captured_node.lock();
      if (!node) {
        return;
      }
      auto descriptors = node->describe_parameters(request->names);
      response->descriptors = descriptors;
    },
    qos_profile
  );

  list_parameters_service_ = node_->create_service<rcl_interfaces::srv::ListParameters>(
    std::string(node_->get_name()) + "/" + parameter_service_names::list_parameters,
    [captured_node](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> response)
    {
      auto node = captured_node.lock();
      if (!node) {
        return;
      }
      auto result = node->list_parameters(request->prefixes, request->depth);
      response->result = result;
    },
    qos_profile
  );
  // *INDENT-ON*
}
