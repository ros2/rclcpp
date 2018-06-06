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

#ifndef RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_INTERFACE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_INTERFACE_HPP_

#include <string>
#include <vector>

#include "rcl_interfaces/msg/list_parameters_result.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Pure virtual interface class for the NodeParameters part of the Node API.
class NodeParametersInterface
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeParametersInterface)

  RCLCPP_PUBLIC
  virtual
  std::vector<rcl_interfaces::msg::SetParametersResult>
  set_parameters(
    const std::vector<rclcpp::Parameter> & parameters) = 0;

  RCLCPP_PUBLIC
  virtual
  rcl_interfaces::msg::SetParametersResult
  set_parameters_atomically(
    const std::vector<rclcpp::Parameter> & parameters) = 0;

  /// Get descriptions of parameters given their names.
  /*
   * \param[in] names a list of parameter names to check.
   * \return the list of parameters that were found.
   * Any parameter not found is omitted from the returned list.
   */
  RCLCPP_PUBLIC
  virtual
  std::vector<rclcpp::Parameter>
  get_parameters(const std::vector<std::string> & names) const = 0;

  /// Get the description of one parameter given a name.
  /*
   * \param[in] name the name of the parameter to look for.
   * \return the parameter if it exists on the node.
   * \throws std::out_of_range if the parameter does not exist on the node.
   */
  RCLCPP_PUBLIC
  virtual
  rclcpp::Parameter
  get_parameter(const std::string & name) const = 0;

  /// Get the description of one parameter given a name.
  /*
   * \param[in] name the name of the parameter to look for.
   * \param[out] parameter the description if parameter exists on the node.
   * \return true if the parameter exists on the node, or
   * \return false if the parameter does not exist.
   */
  RCLCPP_PUBLIC
  virtual
  bool
  get_parameter(
    const std::string & name,
    rclcpp::Parameter & parameter) const = 0;

  RCLCPP_PUBLIC
  virtual
  std::vector<rcl_interfaces::msg::ParameterDescriptor>
  describe_parameters(const std::vector<std::string> & names) const = 0;

  RCLCPP_PUBLIC
  virtual
  std::vector<uint8_t>
  get_parameter_types(const std::vector<std::string> & names) const = 0;

  RCLCPP_PUBLIC
  virtual
  rcl_interfaces::msg::ListParametersResult
  list_parameters(const std::vector<std::string> & prefixes, uint64_t depth) const = 0;

  using ParametersCallbackFunction = std::function<
    rcl_interfaces::msg::SetParametersResult(
      const std::vector<rclcpp::Parameter> &)>;

  RCLCPP_PUBLIC
  virtual
  void
  register_param_change_callback(ParametersCallbackFunction callback) = 0;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_PARAMETERS_INTERFACE_HPP_
