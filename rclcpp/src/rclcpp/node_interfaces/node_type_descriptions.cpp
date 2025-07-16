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

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/node_interfaces/node_type_descriptions.hpp"
#include "rclcpp/parameter_client.hpp"

#include "type_description_interfaces/srv/get_type_description.h"

namespace
{
// Helper wrapper for rclcpp::Service to access ::Request and ::Response types for allocation.
struct GetTypeDescription__C
{
  using Request = type_description_interfaces__srv__GetTypeDescription_Request;
  using Response = type_description_interfaces__srv__GetTypeDescription_Response;
  using Event = type_description_interfaces__srv__GetTypeDescription_Event;
};
}  // namespace

// Helper function for C typesupport.
namespace rosidl_typesupport_cpp
{
template<>
rosidl_service_type_support_t const *
get_service_type_support_handle<GetTypeDescription__C>()
{
  return ROSIDL_GET_SRV_TYPE_SUPPORT(type_description_interfaces, srv, GetTypeDescription);
}
}  // namespace rosidl_typesupport_cpp

namespace rclcpp
{
namespace node_interfaces
{

class NodeTypeDescriptions::NodeTypeDescriptionsImpl
{
public:
  using ServiceT = GetTypeDescription__C;

  rclcpp::Logger logger_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::Service<ServiceT>::SharedPtr type_description_srv_;

  NodeTypeDescriptionsImpl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services)
  : logger_(node_logging->get_logger()),
    node_base_(node_base)
  {
    rclcpp::ParameterValue enable_param;
    const std::string enable_param_name = "start_type_description_service";

    if (!node_parameters->has_parameter(enable_param_name)) {
      enable_param = node_parameters->declare_parameter(
        enable_param_name,
        rclcpp::ParameterValue(true),
        rcl_interfaces::msg::ParameterDescriptor()
        .set__name(enable_param_name)
        .set__type(rclcpp::PARAMETER_BOOL)
        .set__description("Start the ~/get_type_description service for this node.")
        .set__read_only(true));
    } else {
      enable_param = node_parameters->get_parameter(enable_param_name).get_parameter_value();
    }
    if (enable_param.get_type() != rclcpp::PARAMETER_BOOL) {
      RCLCPP_ERROR(
        logger_,
            "Invalid type '%s' for parameter 'start_type_description_service', should be 'bool'",
        rclcpp::to_string(enable_param.get_type()).c_str());
      std::ostringstream ss;
      ss << "Wrong parameter type, parameter {" << enable_param_name << "} is of type {bool}, "
         << "setting it to {" << to_string(enable_param.get_type()) << "} is not allowed.";
      throw rclcpp::exceptions::InvalidParameterTypeException(enable_param_name, ss.str());
    }

    if (enable_param.get<bool>()) {
      auto * rcl_node = node_base->get_rcl_node_handle();
      std::shared_ptr<rcl_service_t> rcl_srv(
        new rcl_service_t,
        [rcl_node, logger = this->logger_](rcl_service_t * service)
        {
          if (rcl_service_fini(service, rcl_node) != RCL_RET_OK) {
            RCLCPP_ERROR(
              logger,
              "Error in destruction of rcl service handle [~/get_type_description]: %s",
              rcl_get_error_string().str);
            rcl_reset_error();
          }
          delete service;
        });
      *rcl_srv = rcl_get_zero_initialized_service();
      rcl_ret_t rcl_ret = rcl_node_type_description_service_init(rcl_srv.get(), rcl_node);

      if (rcl_ret != RCL_RET_OK) {
        RCLCPP_ERROR(
          logger_, "Failed to initialize ~/get_type_description service: %s",
          rcl_get_error_string().str);
        throw std::runtime_error(
                "Failed to initialize ~/get_type_description service.");
      }

      rclcpp::AnyServiceCallback<ServiceT> cb;
      cb.set(
        [this](
          std::shared_ptr<rmw_request_id_t> header,
          std::shared_ptr<ServiceT::Request> request,
          std::shared_ptr<ServiceT::Response> response
        ) {
          rcl_node_type_description_service_handle_request(
            node_base_->get_rcl_node_handle(),
            header.get(),
            request.get(),
            response.get());
        });

      type_description_srv_ = std::make_shared<Service<ServiceT>>(
        node_base_->get_shared_rcl_node_handle(),
        rcl_srv,
        cb);
      node_services->add_service(
        std::dynamic_pointer_cast<ServiceBase>(type_description_srv_),
        nullptr);
    }
  }
};

NodeTypeDescriptions::NodeTypeDescriptions(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services)
: impl_(new NodeTypeDescriptionsImpl(
      node_base,
      node_logging,
      node_parameters,
      node_services))
{}

NodeTypeDescriptions::~NodeTypeDescriptions()
{}

}  // namespace node_interfaces
}  // namespace rclcpp
