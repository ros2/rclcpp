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

#ifndef RCLCPP_RCLCPP_PARAMETER_SERVICE_HPP_
#define RCLCPP_RCLCPP_PARAMETER_SERVICE_HPP_

#include <string>

#include <rmw/rmw.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/get_parameter_types.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

namespace rclcpp
{

namespace parameter_service
{

class ParameterService
{

public:
  RCLCPP_SMART_PTR_DEFINITIONS(ParameterService);

  ParameterService(const rclcpp::node::Node::SharedPtr node)
  : node_(node)
  {
    std::weak_ptr<rclcpp::node::Node> captured_node = node_;
    get_parameters_service_ = node_->create_service<rcl_interfaces::srv::GetParameters>(
      node_->get_name() + "__get_parameters", [captured_node](
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
        }
      );

    get_parameter_types_service_ = node_->create_service<rcl_interfaces::srv::GetParameterTypes>(
      node_->get_name() + "__get_parameter_types", [captured_node](
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
        }
      );

    set_parameters_service_ = node_->create_service<rcl_interfaces::srv::SetParameters>(
      node_->get_name() + "__set_parameters", [captured_node](
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
        }
      );

    set_parameters_atomically_service_ =
      node_->create_service<rcl_interfaces::srv::SetParametersAtomically>(
      node_->get_name() + "__set_parameters_atomically", [captured_node](
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
        }
      );

    describe_parameters_service_ = node_->create_service<rcl_interfaces::srv::DescribeParameters>(
      node_->get_name() + "__describe_parameters", [captured_node](
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
        }
      );

    list_parameters_service_ = node_->create_service<rcl_interfaces::srv::ListParameters>(
      node_->get_name() + "__list_parameters", [captured_node](
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
        }
      );


  }

private:
  const rclcpp::node::Node::SharedPtr node_;
  rclcpp::service::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  rclcpp::service::Service<rcl_interfaces::srv::GetParameterTypes>::SharedPtr
    get_parameter_types_service_;
  rclcpp::service::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;
  rclcpp::service::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr
    set_parameters_atomically_service_;
  rclcpp::service::Service<rcl_interfaces::srv::DescribeParameters>::SharedPtr
    describe_parameters_service_;
  rclcpp::service::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
};

} /* namespace parameter_service */

} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PARAMETER_SERVICE_HPP_ */
