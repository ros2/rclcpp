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

#include "rclcpp/node_interfaces/node_type_descriptions.hpp"

#include "rclcpp/parameter_client.hpp"

#include "type_description_interfaces/srv/get_type_description.h"

#include <memory>
#include <string>

using rclcpp::node_interfaces::NodeTypeDescriptions;

// Even though we just want to use a C service, still need to define a C++-like struct
// for the Service to use to allocate responses/requests.
// There is an allocation and deserialization via the Executor callback mechanism, at least
// of the request, even though we just want to let the C layer take it...
struct GetTypeDescriptionC
{
  using Request = type_description_interfaces__srv__GetTypeDescription_Request;
  using Response = type_description_interfaces__srv__GetTypeDescription_Response;
  using Event = type_description_interfaces__srv__GetTypeDescription_Event;
};

namespace rosidl_typesupport_cpp
{
template<>
rosidl_service_type_support_t const*
get_service_type_support_handle<GetTypeDescriptionC>()
{
  return ROSIDL_GET_SRV_TYPE_SUPPORT(type_description_interfaces, srv, GetTypeDescription);
}
}  // namespace rosidl_typesupport_cpp

namespace rclcpp
{

static constexpr const char * get_type_description_service_name = "get_type_description";


class NodeTypeDescriptions::NodeTypeDescriptionsImpl
{
public:
  bool enabled_ = false;
  Logger logger_;
  node_interfaces::OnSetParametersCallbackHandle param_callback_;
  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>> param_sub_;
  // using ServiceT = type_description_interfaces::srv::GetTypeDescription;
  using ServiceT = GetTypeDescriptionC;

  Service<ServiceT>::SharedPtr type_description_srv_;

  std::shared_ptr<rcl_node_t> rcl_node_;


  NodeTypeDescriptionsImpl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr /* node_topics */)
    : logger_(node_logging->get_logger())
  {
    const std::string enable_param_name = "enable_type_description_service";
    const std::string service_name = "get_type_description";

    rclcpp::ParameterValue enable_param;
    if (!node_parameters->has_parameter(enable_param_name)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.name = enable_param_name;
      descriptor.type = rclcpp::PARAMETER_BOOL;
      descriptor.description = "Enable the ~/get_type_description service for this node.";
      descriptor.read_only = true;
      enable_param = node_parameters->declare_parameter(
        enable_param_name, rclcpp::ParameterValue(true));
    } else {
      enable_param = node_parameters->get_parameter(enable_param_name).get_parameter_value();
    }
    if (enable_param.get_type() == rclcpp::PARAMETER_BOOL) {
      if (enable_param.get<bool>()) {
        enabled_ = true;
      }
    } else {
      RCLCPP_ERROR(
        logger_, "Invalid type '%s' for parameter '%s', should be 'bool'",
        rclcpp::to_string(enable_param.get_type()).c_str(), enable_param_name.c_str());
      throw std::invalid_argument(
        "Invalid type for parameter '" + enable_param_name + "', should be bool");
    }

    if (enabled_) {
      rcl_node_ = node_base->get_shared_rcl_node_handle();
      rcl_ret_t rcl_ret = rcl_node_type_description_service_init(rcl_node_.get());
      if (rcl_ret != RCL_RET_OK) {
        RCLCPP_ERROR(
          logger_, "Failed to initialize ~/get_type_description_service: %s",
          rcl_get_error_string().str);
        throw std::runtime_error(
          "Failed to initialize ~/get_type_description service.");
      }

      rcl_service_t * rcl_srv = NULL;
      rcl_ret = rcl_node_get_type_description_service(rcl_node_.get(), &rcl_srv);
      if (rcl_ret != RCL_RET_OK) {
        throw std::runtime_error(
          "Failed to get initialized ~/get_type_description service from rcl.");
      }

      rclcpp::AnyServiceCallback<GetTypeDescriptionC> cb;
      cb.set([this]() {
        RCLCPP_WARN(logger_, "SERVICE CALLBACK");
        rcl_node_type_description_service_on_new_request(rcl_node_.get());
      });

      type_description_srv_ = std::make_shared<Service<ServiceT>>(
        rcl_node_,
        rcl_srv,
        cb
        // [this, &rcl_node]() {
        //   RCLCPP_ERROR(logger_, "SERBICE");
        //   // rcl_node_type_description_service_on_new_request(rcl_node.get());
        // }
      );
      node_services->add_service(
        std::dynamic_pointer_cast<ServiceBase>(type_description_srv_), nullptr);
    }

    // param_callback_ = node_parameters->add_on_set_parameters_callback(
    //   std::bind(&NodeTypeDescriptionsImpl::on_set_parameters, this, std::placeholders::_1));
    // // TODO(tfoote) use parameters interface not subscribe to events via topic ticketed #609
    // param_sub_ = rclcpp::AsyncParametersClient::on_parameter_event(
    //   node_topics,
    //   std::bind(&NodeTypeDescriptionsImpl::on_parameter_event, this, std::placeholders::_1));
  }
};


NodeTypeDescriptions::NodeTypeDescriptions(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics)
:
  // node_base_(node_base),
  // node_services_(node_services),
  // node_logging_(node_logging),
  // node_parameters_(node_parameters)
  impl_(new NodeTypeDescriptionsImpl(
    node_base,
    node_logging,
    node_parameters,
    node_services,
    node_topics
  ))
{
  // TODO init rcl node type description service and attach callback
}

NodeTypeDescriptions::~NodeTypeDescriptions()
{
  RCUTILS_LOG_WARN("Destroy TDimpl");
}

}  // namespace rclcpp
