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

/// Encapsulation of options for initializing node.
class NodeOptions
{
public:
  // use this calss to construct NodeOptions.
  class Builder;

public:
  /// Copy constructor.
  RCLCPP_PUBLIC
  NodeOptions(const NodeOptions & other);

  /// Assignment operator.
  RCLCPP_PUBLIC
  NodeOptions &
  operator=(const NodeOptions & other);

  /// Destructor
  RCLCPP_PUBLIC
  virtual
  ~NodeOptions();

  /// Return the rcl_node_options used by the node.
  RCLCPP_PUBLIC
  const rcl_node_options_t *
  get_rcl_node_options() const;

  /// Return the context to be used by the node
  RCLCPP_PUBLIC
  rclcpp::Context::SharedPtr
  context() const;

  /// Return a reference to the list of initial parameters
  RCLCPP_PUBLIC
  const std::vector<rclcpp::Parameter> &
  initial_parameters() const;

  /// Return a reference to the use_global_arguments flag
  RCLCPP_PUBLIC
  const bool &
  use_global_arguments() const;

  /// Return a reference to the use_intra_process_comms flag
  RCLCPP_PUBLIC
  const bool &
  use_intra_process_comms() const;

  /// Return a reference to the start_parameter_services flag
  RCLCPP_PUBLIC
  const bool &
  start_parameter_services() const;

protected:
  /// Create NodeOptions, optionally specifying the allocator to use.
  /**
   * \param[in] context The context for the node (usually represents the state of a process).
   * \param[in] arguments Command line arguments that should apply only to this node.
   * This can be used to provide remapping rules that only affect one instance.
   * \param[in] initial_parameters a list of initial values for the parameters on the node.
   * \param[in] use_global_arguments False to prevent node using arguments passed to the process.
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   * \param[in] start_parameter_services True to setup ROS interfaces for accessing parameters
   * in the node.
   * \param[in] allocator allocator to use in construction of NodeOptions.
   */
  RCLCPP_PUBLIC
  explicit NodeOptions(
    rclcpp::Context::SharedPtr context,
    const std::vector<std::string> & arguments,
    const std::vector<rclcpp::Parameter> & initial_parameters,
    bool use_global_arguments,
    bool use_intra_process_comms,
    bool start_parameter_services,
    rcl_allocator_t allocator);

  /// Clean up internal rcl_node_options_t structure.
  void
  finalize_node_options();

  /// Retrieve the ROS_DOMAIN_ID environment variable and populate options.
  size_t
  get_domain_id_from_env() const;

private:
  /// Underlying rcl_node_options structure
  std::unique_ptr<rcl_node_options_t> node_options_;

  /// The context for the node
  rclcpp::Context::SharedPtr context_;

  /// Initial parameters to set on the node
  std::vector<rclcpp::Parameter> initial_parameters_;

  /// Use the optimized intra-process communication pipeline.
  bool use_intra_process_comms_;

  /// Start ROS interfaces for accessing parameters in the node.
  bool start_parameter_services_;
};

/// Builder helper for creating NodeOptions instances
/**
 * The Builder object provides a mechanism for creating NodeOptions instances with particular
 * default values overridden. It exposes all of the potential options for a Node instance.
 */
class NodeOptions::Builder
{
public:
  /// Constructor
  Builder() = default;

  /// Using the current state of the Builder, create a NodeOptions instance.
  NodeOptions build();

  /// Set the context for the node instance to use
  /**
   * The context defaults to the global default context.
   * \param[in] context The context for the node to use
   */
  Builder &
  context(rclcpp::Context::SharedPtr context);

  /// Set the local arguments for the node instance to use
  /**
   * Defaults to an empty set of arguments
   * \param[in] arguments The list of arguments to be used
   */
  Builder &
  arguments(const std::vector<std::string> & arguments);

  /// Set the initial parameter values to set on the node instance
  /**
   * Defaults to an empty set of parameter values
   * \param[in] initial_parameters a list of initial values for the parameters on the node.
   */
  Builder &
  initial_parameters(const std::vector<rclcpp::Parameter> initial_parameters);

  /// Set whether the node uses or ignores process-wide arguments
  /**
   * Defaults to true (uses global arguments)
   * \param[in] use_global_arguments False to prevent node using arguments passed to the process.
   */
  Builder &
  use_global_arguments(bool use_global_arguments);

  /// Set whether the node uses the optimized intra-process pipeline.
  /**
   * Defaults to false (does not use shared memory intra-process communications)
   * \param[in] use_intra_process_comms True to use the optimized intra-process communication
   * pipeline to pass messages between nodes in the same process using shared memory.
   */
  Builder &
  use_intra_process_comms(bool use_intra_process_comms);

  /// Set whether the parameter services will be started on the node instance.
  /**
   * Defaults to true (parameter services will be started)
   * \param[in] start_parameter_services True to setup ROS interfaces for accessing parameters
   * in the node.
   */
  Builder &
  start_parameter_services(bool start_parameter_services);

  /// Set the allocator to be used by NodeOptions and the node instance
  /**
   * Defaults to the rcl_default_allocator.
   * \param[in] allocator allocator to use in construction of NodeOptions.
   */
  Builder &
  allocator(rcl_allocator_t allocator);

private:
  // IMPORTANT: if any of these default values are changed, please update the
  // documentation on the corresponding Builder methods above to reflect the changes.

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
