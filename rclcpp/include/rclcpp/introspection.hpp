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


#ifndef RCLCPP__INTROSPECTION_HPP_
#define RCLCPP__INTROSPECTION_HPP_

#include "rclcpp/client.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/service.hpp" // this smells circular
#include "rcl/node.h"
#include "rcl/service.h"
#include "rcl/client.h"
#include "rcl/introspection.h"

namespace rclcpp
{

/* Provides on-parameter-change configuration features to service introspection
 *
 *
 */
class IntrospectionUtils
{
  public:

    explicit IntrospectionUtils(
        rcl_node_t * rcl_node_ptr,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters);
    
    explicit IntrospectionUtils(
        const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
        const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& node_parameters)
      : IntrospectionUtils(node_base->get_rcl_node_handle(), node_parameters){};

    virtual ~IntrospectionUtils();
    void register_service(const rclcpp::ServiceBase::SharedPtr& service);
    void register_client(const rclcpp::ClientBase::SharedPtr& client);


    
  private:
    std::vector<rcl_service_t *> services;
    std::vector<rcl_client_t *> clients;
    rcl_node_t * rcl_node_ptr_;
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_;
};
  





} // namespace rclcpp
#endif // RCLCPP__INTROSPECTION_HPP_
