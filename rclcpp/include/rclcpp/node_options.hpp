// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_OPTIONS_HPP_
#define RCLCPP__NODE_OPTIONS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rcl/node_options.h"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Encapsulation of options for node initialization.
class NodeOptions
{
public:
  /// Create NodeOptions with default values, optionally specifying the allocator to use.
  /**
   * Default values for the node options:
   *
   *   - context = rclcpp::contexts::default_context::get_global_default_context()
   *   - arguments = {}
   *   - initial_parameters = {}
   *   - use_global_arguments = true
   *   - use_intra_process_comms = false
   *   - start_parameter_services = true
   *   - allocator = rcl_get_default_allocator()
   *
   * \param[in] allocator allocator to use in construction of NodeOptions.
   */
  RCLCPP_PUBLIC
  explicit NodeOptions(rcl_allocator_t allocator = rcl_get_default_allocator());

  /// Destructor.
  RCLCPP_PUBLIC
  virtual
  ~NodeOptions() = default;

  /// Copy constructor.
  RCLCPP_PUBLIC
  NodeOptions(const NodeOptions & other);

  /// Assignment operator.
  RCLCPP_PUBLIC
  NodeOptions &
  operator=(const NodeOptions & other);

  /// Return the rcl_node_options used by the node.
  /**
   * This data structure is created lazily, on the first call to this function.
   * Repeated calls will not regenerate it unless one of the input settings
   * changed, like arguments, use_global_arguments, or the rcl allocator.
   */
  RCLCPP_PUBLIC
  const rcl_node_options_t *
  get_rcl_node_options() const;

  /// Return the context to be used by the node.
  RCLCPP_PUBLIC
  rclcpp::Context::SharedPtr
  context() const;

  /// Set the context, return this for parameter idiom.
  RCLCPP_PUBLIC
  NodeOptions &
  context(rclcpp::Context::SharedPtr context);

  /// Return a reference to the list of arguments for the node.
  RCLCPP_PUBLIC
  const std::vector<std::string> &
  arguments() const;

  /// Set the arguments, return this for parameter idiom.
  /**
   * This will cause the internal rcl_node_options_t struct to be invalidated.
   */
  RCLCPP_PUBLIC
  NodeOptions &
  arguments(const std::vector<std::string> & arguments);

  /// Return a reference to the list of initial parameters.
  RCLCPP_PUBLIC
  std::vector<rclcpp::Parameter> &
  initial_parameters();

  RCLCPP_PUBLIC
  const std::vector<rclcpp::Parameter> &
  initial_parameters() const;

  /// Set the initial parameters, return this for parameter idiom.
  RCLCPP_PUBLIC
  NodeOptions &
  initial_parameters(const std::vector<rclcpp::Parameter> & initial_parameters);

  /// Append a single initial parameter, parameter idiom style.
  template<typename ParameterT>
  NodeOptions &
  append_initial_parameter(const std::string & name, const ParameterT & value)
  {
    this->initial_parameters().emplace_back(name, rclcpp::ParameterValue(value));
    return *this;
  }

  /// Return a reference to the use_global_arguments flag.
  RCLCPP_PUBLIC
  const bool &
  use_global_arguments() const;

  /// Set the use_global_arguments flag, return this for parameter idiom.
  /**
   * This will cause the internal rcl_node_options_t struct to be invalidated.
   */
  RCLCPP_PUBLIC
  NodeOptions &
  use_global_arguments(const bool & use_global_arguments);

  /// Return a reference to the use_intra_process_comms flag
  RCLCPP_PUBLIC
  const bool &
  use_intra_process_comms() const;

  /// Set the use_intra_process_comms flag, return this for parameter idiom.
  RCLCPP_PUBLIC
  NodeOptions &
  use_intra_process_comms(const bool & use_intra_process_comms);

  /// Return a reference to the start_parameter_services flag
  RCLCPP_PUBLIC
  const bool &
  start_parameter_services() const;

  /// Set the start_parameter_services flag, return this for parameter idiom.
  RCLCPP_PUBLIC
  NodeOptions &
  start_parameter_services(const bool & start_parameter_services);

  /// Return the rcl_allocator_t to be used.
  RCLCPP_PUBLIC
  const rcl_allocator_t &
  allocator() const;

  /// Set the rcl_allocator_t to be used, may cause deallocation of existing rcl_node_options_t.
  /**
   * This will cause the internal rcl_node_options_t struct to be invalidated.
   */
  RCLCPP_PUBLIC
  NodeOptions &
  allocator(rcl_allocator_t allocator);

protected:
  /// Retrieve the ROS_DOMAIN_ID environment variable and populate options.
  size_t
  get_domain_id_from_env() const;

private:
  // This is mutable to allow for a const accessor which lazily creates the node options instance.
  /// Underlying rcl_node_options structure.
  mutable std::unique_ptr<rcl_node_options_t, void (*)(rcl_node_options_t *)> node_options_;

  // IMPORTANT: if any of these default values are changed, please update the
  // documentation in this class.

  rclcpp::Context::SharedPtr context_ {
    rclcpp::contexts::default_context::get_global_default_context()};

  std::vector<std::string> arguments_ {};

  std::vector<rclcpp::Parameter> initial_parameters_ {};

  bool use_global_arguments_ {true};

  bool use_intra_process_comms_ {false};

  bool start_parameter_services_ {true};

  rcl_allocator_t allocator_ {rcl_get_default_allocator()};
};

}  // namespace rclcpp

#endif  // RCLCPP__NODE_OPTIONS_HPP_
