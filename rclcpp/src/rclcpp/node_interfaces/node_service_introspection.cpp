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

#include <functional>
#include "rclcpp/node_interfaces/node_service_introspection.hpp"
#include "rcl/service_introspection.h"
#include "rcl/client.h"
#include "rcl/service.h"

using rclcpp::node_interfaces::NodeServiceIntrospection;


NodeServiceIntrospection::NodeServiceIntrospection(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters)
: node_base_(node_base)
{
  // declare service introspection parameters
  if (!node_parameters->has_parameter("publish_service_events")) {
    node_parameters->declare_parameter("publish_service_events", rclcpp::ParameterValue(true));
  }
  if (!node_parameters->has_parameter("publish_service_content")) {
    node_parameters->declare_parameter("publish_service_content", rclcpp::ParameterValue(true));
  }
  if (!node_parameters->has_parameter("publish_client_events")) {
    node_parameters->declare_parameter("publish_client_events", rclcpp::ParameterValue(true));
  }
  if (!node_parameters->has_parameter("publish_client_content")) {
    node_parameters->declare_parameter("publish_client_content", rclcpp::ParameterValue(true));
  }

  std::function<void(const std::vector<rclcpp::Parameter> &)>
  configure_service_introspection_callback =
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_ret_t ret;
      for (const auto & param : parameters) {
        if (param.get_name() == "publish_service_events") {
          for (auto srv = services_.begin(); srv != services_.end(); ++srv) {
            if (srv->expired()) {
              srv = services_.erase(srv);
            } else {
              ret = rcl_service_introspection_configure_server_service_events(
                srv->lock()->get_service_handle().get(),
                this->node_base_->get_rcl_node_handle(),
                param.get_value<bool>());
              if (RCL_RET_OK != ret) {
                throw std::runtime_error(
                        std::string(
                          "Failed to configure service introspection events with error ") +
                        std::to_string(ret));
              }
            }
          }
        } else if (param.get_name() == "publish_client_events") {
          for (auto clt = clients_.begin(); clt != clients_.end(); ++clt) {
            if (clt->expired()) {
              clt = clients_.erase(clt);
            } else {
              ret = rcl_service_introspection_configure_client_service_events(
                clt->lock()->get_client_handle().get(),
                this->node_base_->get_rcl_node_handle(),
                param.get_value<bool>());
              if (RCL_RET_OK != ret) {
                throw std::runtime_error(
                        std::string(
                          "Failed to configure service introspection events with error ") +
                        std::to_string(ret));
              }
            }
          }
        } else if (param.get_name() == "publish_service_content") {
          for (auto srv = services_.begin(); srv != services_.end(); ++srv) {
            if (srv->expired()) {
              srv = services_.erase(srv);
            } else {
              ret = rcl_service_introspection_configure_server_service_event_message_payload(
                srv->lock()->get_service_handle().get(), param.get_value<bool>());
              if (RCL_RET_OK != ret) {
                throw std::runtime_error(
                        std::string(
                          "Failed to configure service introspection message payload with error ") +
                        std::to_string(ret));
              }
            }
          }
        } else if (param.get_name() == "publish_client_content") {
          for (auto clt = clients_.begin(); clt != clients_.end(); ++clt) {
            if (clt->expired()) {
              clt = clients_.erase(clt);
            } else {
              ret = rcl_service_introspection_configure_client_service_event_message_payload(
                clt->lock()->get_client_handle().get(), param.get_value<bool>());
              if (RCL_RET_OK != ret) {
                throw std::runtime_error(
                        std::string(
                          "Failed to configure service introspection message payload with error ") +
                        std::to_string(ret));
              }
            }
          }
        }
      }
    };
  // register callbacks
  post_set_parameters_callback_handle_ = node_parameters->add_post_set_parameters_callback(
    configure_service_introspection_callback);
}

size_t
NodeServiceIntrospection::register_client(rclcpp::ClientBase::SharedPtr client)
{
  std::weak_ptr<rclcpp::ClientBase> weak_client = client;
  clients_.push_back(weak_client);
  return clients_.size();
}

size_t
NodeServiceIntrospection::register_service(rclcpp::ServiceBase::SharedPtr service)
{
  std::weak_ptr<rclcpp::ServiceBase> weak_service = service;
  services_.push_back(weak_service);
  return services_.size();
}
