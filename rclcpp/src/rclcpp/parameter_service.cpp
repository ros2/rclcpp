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

#include "rclcpp/logging.hpp"

#include "./parameter_service_names.hpp"

using rclcpp::ParameterService;

ParameterService::ParameterService(
  const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base,
  const std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services,
  rclcpp::node_interfaces::NodeParametersInterface * node_params,
  const rmw_qos_profile_t & qos_profile)
{
  const std::string node_name = node_base->get_name();

  get_parameters_service_ = create_service<rcl_interfaces::srv::GetParameters>(
    node_base, node_services,
    node_name + "/" + parameter_service_names::get_parameters,
    [node_params](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response)
    {
      try {
        auto parameters = node_params->get_parameters(request->names);
        for (const auto & param : parameters) {
          response->values.push_back(param.get_value_message());
        }
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Failed to get parameters: %s", ex.what());
      }
    },
    qos_profile, nullptr);

  get_parameter_types_service_ = create_service<rcl_interfaces::srv::GetParameterTypes>(
    node_base, node_services,
    node_name + "/" + parameter_service_names::get_parameter_types,
    [node_params](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Request> request,
      std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Response> response)
    {
      try {
        auto types = node_params->get_parameter_types(request->names);
        std::transform(
          types.cbegin(), types.cend(),
          std::back_inserter(response->types), [](const uint8_t & type) {
            return static_cast<rclcpp::ParameterType>(type);
          });
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Failed to get parameter types: %s", ex.what());
      }
    },
    qos_profile, nullptr);

  set_parameters_service_ = create_service<rcl_interfaces::srv::SetParameters>(
    node_base, node_services,
    node_name + "/" + parameter_service_names::set_parameters,
    [node_params](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response)
    {
      // Set parameters one-by-one, since there's no way to return a partial result if
      // set_parameters() fails.
      auto result = rcl_interfaces::msg::SetParametersResult();
      for (auto & p : request->parameters) {
        try {
          result = node_params->set_parameters_atomically(
            {rclcpp::Parameter::from_parameter_msg(p)});
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Failed to set parameter: %s", ex.what());
          result.successful = false;
          result.reason = ex.what();
        }
        response->results.push_back(result);
      }
    },
    qos_profile, nullptr);

  set_parameters_atomically_service_ = create_service<rcl_interfaces::srv::SetParametersAtomically>(
    node_base, node_services,
    node_name + "/" + parameter_service_names::set_parameters_atomically,
    [node_params](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> response)
    {
      std::vector<rclcpp::Parameter> pvariants;
      std::transform(
        request->parameters.cbegin(), request->parameters.cend(),
        std::back_inserter(pvariants),
        [](const rcl_interfaces::msg::Parameter & p) {
          return rclcpp::Parameter::from_parameter_msg(p);
        });
      try {
        auto result = node_params->set_parameters_atomically(pvariants);
        response->result = result;
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("rclcpp"), "Failed to set parameters atomically: %s", ex.what());
        response->result.successful = false;
        response->result.reason = "One or more parameters were not declared before setting";
      }
    },
    qos_profile, nullptr);

  describe_parameters_service_ = create_service<rcl_interfaces::srv::DescribeParameters>(
    node_base, node_services,
    node_name + "/" + parameter_service_names::describe_parameters,
    [node_params](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Response> response)
    {
      try {
        auto descriptors = node_params->describe_parameters(request->names);
        response->descriptors = descriptors;
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException & ex) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Failed to describe parameters: %s", ex.what());
      }
    },
    qos_profile, nullptr);

  list_parameters_service_ = create_service<rcl_interfaces::srv::ListParameters>(
    node_base, node_services,
    node_name + "/" + parameter_service_names::list_parameters,
    [node_params](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> response)
    {
      auto result = node_params->list_parameters(request->prefixes, request->depth);
      response->result = result;
    },
    qos_profile, nullptr);
}
