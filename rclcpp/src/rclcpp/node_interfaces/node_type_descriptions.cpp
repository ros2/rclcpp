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
  using ServiceT = GetTypeDescriptionC;
  Service<ServiceT>::SharedPtr type_description_srv_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;

  std::thread type_description_service_thread_;
  rclcpp::CallbackGroup::SharedPtr type_description_service_callback_group_;
  rclcpp::executors::SingleThreadedExecutor type_description_service_executor_;

  NodeTypeDescriptionsImpl(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr /* node_topics */)
    : logger_(node_logging->get_logger()),
      node_base_(node_base),
      type_description_service_callback_group_(
        node_base_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false))
  {
    const std::string enable_param_name = "enable_type_description_service";
    const std::string service_name = "get_type_description";

    bool enabled_;
    try {
      auto enable_param = node_parameters->declare_parameter(
        enable_param_name,
        rclcpp::ParameterValue(true),
        rcl_interfaces::msg::ParameterDescriptor()
          .set__name(enable_param_name)
          .set__type(rclcpp::PARAMETER_BOOL)
          .set__description("Enable the ~/get_type_description service for this node.")
          .set__read_only(true));
      enabled_ = enable_param.get<bool>();
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & exc) {
      RCLCPP_ERROR(logger_, "%s", exc.what());
      throw;
    }

    if (enabled_) {
      auto rcl_node = node_base->get_rcl_node_handle();
      rcl_ret_t rcl_ret = rcl_node_type_description_service_init(rcl_node);
      if (rcl_ret != RCL_RET_OK) {
        RCLCPP_ERROR(
          logger_, "Failed to initialize ~/get_type_description_service: %s",
          rcl_get_error_string().str);
        throw std::runtime_error(
          "Failed to initialize ~/get_type_description service.");
      }

      rcl_service_t * rcl_srv = nullptr;
      rcl_ret = rcl_node_get_type_description_service(rcl_node, &rcl_srv);
      if (rcl_ret != RCL_RET_OK) {
        throw std::runtime_error(
          "Failed to get initialized ~/get_type_description service from rcl.");
      }

      rclcpp::AnyServiceCallback<ServiceT> cb;
      cb.set([this](
          std::shared_ptr<rmw_request_id_t> header,
          std::shared_ptr<ServiceT::Request> request,
          std::shared_ptr<ServiceT::Response> response
      ) {
        RCLCPP_WARN(logger_, "SERVICE CALLBACK");
        rcl_node_type_description_service_handle_request(
          node_base_->get_rcl_node_handle(),
          *header,
          request.get(),
          response.get());
      });

      type_description_srv_ = std::make_shared<Service<ServiceT>>(
        node_base_->get_shared_rcl_node_handle(),
        rcl_srv,
        cb);
      node_services->add_service(
        std::dynamic_pointer_cast<ServiceBase>(type_description_srv_),
        type_description_service_callback_group_);

      type_description_service_thread_ = std::thread([this]() {
        type_description_service_executor_.add_callback_group(
          type_description_service_callback_group_,
          node_base_);
        type_description_service_executor_.spin();
      });
    }
  }

  virtual ~NodeTypeDescriptionsImpl() {
    type_description_service_executor_.cancel();
    type_description_service_thread_.join();
  }
};


NodeTypeDescriptions::NodeTypeDescriptions(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics)
: impl_(new NodeTypeDescriptionsImpl(
    node_base,
    node_logging,
    node_parameters,
    node_services,
    node_topics))
{}

NodeTypeDescriptions::~NodeTypeDescriptions()
{}

}  // namespace rclcpp
