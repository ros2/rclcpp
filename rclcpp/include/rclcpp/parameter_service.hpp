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

#include <rcl_interfaces/GetParameters.h>
#include <rcl_interfaces/GetParameterTypes.h>
#include <rcl_interfaces/Parameter.h>
#include <rcl_interfaces/ParameterDescriptor.h>
#include <rcl_interfaces/ParameterType.h>
#include <rcl_interfaces/SetParameters.h>
#include <rcl_interfaces/SetParametersAtomically.h>
#include <rcl_interfaces/ListParameters.h>
#include <rcl_interfaces/DescribeParameters.h>

namespace rclcpp
{

namespace parameter_service
{

class ParameterService
{

public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(ParameterService);

  ParameterService(const rclcpp::node::Node::SharedPtr & node)
  : node_(node)
  {
    get_parameters_service_ = node_->create_service<rcl_interfaces::GetParameters>(
      node_->get_name() + "__get_parameters", [&node](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::GetParameters::Request> request,
        std::shared_ptr<rcl_interfaces::GetParameters::Response> response)
        {
          auto values = node->get_parameters(request->names);
          std::transform(values.cbegin(), values.cend(), std::back_inserter(response->values),
          [](const rclcpp::parameter::ParameterVariant & pvariant) {
            return pvariant.
            get_parameter_value();
          });
        }
      );

    get_parameter_types_service_ = node_->create_service<rcl_interfaces::GetParameterTypes>(
      node_->get_name() + "__get_parameter_types", [&node](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::GetParameterTypes::Request> request,
        std::shared_ptr<rcl_interfaces::GetParameterTypes::Response> response)
        {
          auto types = node->get_parameter_types(request->parameter_names);
          std::transform(types.cbegin(), types.cend(),
          std::back_inserter(response->parameter_types), [](const uint8_t & type) {
            return static_cast<rclcpp::parameter::ParameterType>(type);
          });
        }
      );

    set_parameters_service_ = node_->create_service<rcl_interfaces::SetParameters>(
      node_->get_name() + "__set_parameters", [&node](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::SetParameters::Request> request,
        std::shared_ptr<rcl_interfaces::SetParameters::Response> response)
        {
          std::vector<rclcpp::parameter::ParameterVariant> pvariants;
          std::transform(request->parameters.cbegin(), request->parameters.cend(),
          std::back_inserter(pvariants),
          [](const rcl_interfaces::Parameter & p) {
            return rclcpp::parameter::ParameterVariant::
            from_parameter(p);
          });
          auto results = node->set_parameters(pvariants);
          response->results = results;
        }
      );

    set_parameters_atomically_service_ =
      node_->create_service<rcl_interfaces::SetParametersAtomically>(
      node_->get_name() + "__set_parameters_atomically", [&node](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::SetParametersAtomically::Request> request,
        std::shared_ptr<rcl_interfaces::SetParametersAtomically::Response> response)
        {
          std::vector<rclcpp::parameter::ParameterVariant> pvariants;
          std::transform(request->parameters.cbegin(), request->parameters.cend(),
          std::back_inserter(pvariants),
          [](const rcl_interfaces::Parameter & p) {
            return rclcpp::parameter::ParameterVariant::
            from_parameter(p);
          });
          auto result = node->set_parameters_atomically(pvariants);
          response->result = result;
        }
      );

    describe_parameters_service_ = node_->create_service<rcl_interfaces::DescribeParameters>(
      node_->get_name() + "__describe_parameters", [&node](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::DescribeParameters::Request> request,
        std::shared_ptr<rcl_interfaces::DescribeParameters::Response> response)
        {
          auto descriptors = node->describe_parameters(request->names);
          response->descriptors = descriptors;
        }
      );

    list_parameters_service_ = node_->create_service<rcl_interfaces::ListParameters>(
      node_->get_name() + "__describe_parameters", [&node](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rcl_interfaces::ListParameters::Request> request,
        std::shared_ptr<rcl_interfaces::ListParameters::Response> response)
        {
          auto result = node->list_parameters(request->parameter_prefixes, request->depth);
          response->result = result;
        }
      );


  }

private:
  const rclcpp::node::Node::SharedPtr node_;
  rclcpp::service::Service<rcl_interfaces::GetParameters>::SharedPtr get_parameters_service_;
  rclcpp::service::Service<rcl_interfaces::GetParameterTypes>::SharedPtr
    get_parameter_types_service_;
  rclcpp::service::Service<rcl_interfaces::SetParameters>::SharedPtr set_parameters_service_;
  rclcpp::service::Service<rcl_interfaces::SetParametersAtomically>::SharedPtr
    set_parameters_atomically_service_;
  rclcpp::service::Service<rcl_interfaces::DescribeParameters>::SharedPtr
    describe_parameters_service_;
  rclcpp::service::Service<rcl_interfaces::ListParameters>::SharedPtr list_parameters_service_;
};

} /* namespace parameter_service */

} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_PARAMETER_SERVICE_HPP_ */
