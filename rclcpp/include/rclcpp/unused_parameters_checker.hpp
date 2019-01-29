// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__UNUSED_PARAMETERS_CHECKER_HPP_
#define RCLCPP__UNUSED_PARAMETERS_CHECKER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/node_interfaces/get_node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Check a Node-like class for unused parameters.
/**
 * This class can be used to detect misconfigurations and typos by ensuring all
 * initial parameter values that were passed to the Node-like object were used.
 * So this class's methods are used after or during Node construction and after
 * all parameters have been declared.
 *
 * This class must not outlive the Node that it's being used with.
 */
class UnusedParametersChecker
{
public:
  template<typename NodeType>
  explicit UnusedParametersChecker(NodeType && node_like)
  : node_parameters_interface_(
      rclcpp::node_interfaces::get_node_parameters_interface(std::forward<NodeType>(node_like)))
  {}

  /// Warn if any initial parameter values have not been used.
  /**
   * This function will complain with a RCLCPP_WARN if any provided initial
   * parameter values have not been used.
   *
   * \throws std::bad_alloc when trying to create an error message
   */
  RCLCPP_PUBLIC
  void
  check_and_warn() const;

  /// Throw an UnusedParameterExecption if any initial parameter values have not been used.
  /**
   * This function will throw an exception if any provided initial
   * parameter values have not been used.
   *
   * \throws std::bad_alloc when trying to create an error message
   * \throws rclcpp::UnusedParametersException when there are unused parameters
   */
  RCLCPP_PUBLIC
  void
  check_and_throw() const;

  /// Return the number of unused initial parameter values.
  /**
   * Similar to get_unused_initial_parameter_values(), but it returns the
   * number of unused parameter values rather than a vector of the unused
   * parameters (which involves allocating storage for the copies).
   * This function is faster and avoids memory allocation while checking for a
   * problem, and if one is detected then get_unused_initial_parameter_values()
   * may be used to format a useful error message.
   *
   * \returns the number of unused initial parameter values
   */
  RCLCPP_PUBLIC
  size_t
  count_unused_initial_parameter_values() const;

  /// Return the list of unused initial parameter values.
  /**
   * A common case where this returns a non-empty vector, is when someone makes
   * a typo when setting the parameters from outside the node.
   * For example, if they use `ip_addr` rather than the expected `ip_address`.
   *
   * \returns vector of parameters which where passed to the node but where
   *   not declared before this function was called.
   * \throws std::bad_alloc
   */
  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter>
  get_unused_initial_parameter_values() const;

private:
  const rclcpp::node_interfaces::NodeParametersInterface * node_parameters_interface_;
};

/// Thrown when throw_if_unused_initialized_parameter_values() finds unused parameters.
class UnusedParametersException : public std::runtime_error
{
public:
  explicit UnusedParametersException(const std::vector<rclcpp::Parameter> & unused_parameters);
};

}  // namespace rclcpp

#endif  // RCLCPP__UNUSED_PARAMETERS_CHECKER_HPP_
