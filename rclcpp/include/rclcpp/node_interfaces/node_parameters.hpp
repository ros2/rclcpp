// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeParameters part of the Node API.
class NodeParameters : public NodeParametersInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeParameters)

  RCLCPP_PUBLIC
  NodeParameters(
    const node_interfaces::NodeBaseInterface::SharedPtr node_base,
    const node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    const node_interfaces::NodeServicesInterface::SharedPtr node_services,
    const std::vector<Parameter> & initial_parameters,
    bool use_intra_process,
    bool start_parameter_services);

  RCLCPP_PUBLIC
  virtual
  ~NodeParameters();

  RCLCPP_PUBLIC
  virtual
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  RCLCPP_PUBLIC
  virtual
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters);

  RCLCPP_PUBLIC
  virtual
  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  virtual
  rclcpp::Parameter
  get_parameter(const std::string & name) const;

  RCLCPP_PUBLIC
  virtual
  bool
  get_parameter(
    const std::string & name,
    rclcpp::Parameter & parameter) const;

  RCLCPP_PUBLIC
  virtual
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  virtual
  std::vector<uint8_t>
  get_parameter_types(const std::vector<std::string> & names) const;

  RCLCPP_PUBLIC
  virtual
  rcl_interfaces::msg::ListParametersResult
  list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const;

  RCLCPP_PUBLIC
  virtual
  void
  register_param_change_callback(ParametersCallbackFunction callback);

private:
  RCLCPP_DISABLE_COPY(NodeParameters)

  mutable std::mutex mutex_;

  ParametersCallbackFunction parameters_callback_ = nullptr;

  std::map<std::string, rclcpp::Parameter> parameters_;

  Publisher<rcl_interfaces::msg::ParameterEvent>::SharedPtr events_publisher_;

  std::shared_ptr<ParameterService> parameter_service_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_HPP_
