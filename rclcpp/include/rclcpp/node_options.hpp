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
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Encapsulation of options for initializing node.
class NodeOptions
{
public:
  /// Create NodeOptions, optionally specifying the allocator to use.
  /**
   * \param[in] allocator allocator to use in construction of NodeOptions.
   */
  RCLCPP_PUBLIC
  explicit NodeOptions(
    rcl_allocator_t allocator = rcl_get_default_allocator());

  /// Create NodeOptions, optionally specifying the allocator to use.
  /**
   * \param[in] arguments Command line arguments that should apply only to this node.
   * \param[in] initial_parameters a list of initial values for parameters on the node.
   * This can be used to provide remapping rules that only affect one instance.
   * \param[in] allocator - allocator to use in construction of NodeOptions.
   */
  RCLCPP_PUBLIC
  explicit NodeOptions(
    const std::vector<std::string> & arguments,
    const std::vector<rclcpp::Parameter> & initial_parameters = {},
    rcl_allocator_t allocator = rcl_get_default_allocator());

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

  /// Return the rcl node options.
  RCLCPP_PUBLIC
  const rcl_node_options_t *
  get_rcl_node_options() const;

  /// Return a reference to the list of initial parameters
  /**
   * Initial paramters is a list of initial values for parameters on the node.
   * This can be used to provide remapping rules that only affect the node instance created with
   * this NodeOptions instance.
   */
  const std::vector<rclcpp::Parameter> & initial_parameters() const;
  std::vector<rclcpp::Parameter> & initial_parameters();

  /// Return a reference to the use_global_arguments flag
  /**
   * False to prevent node from using arguments passed to the process
   */
  const bool & use_global_arguments() const;
  bool & use_global_arguments();

  /// Return a reference to the use_intra_process_comms flag
  /**
   * True to use the optimized intra-process communication pipeline to pass messages bewteen nodes
   * in the same process using shared memory.
   */
  const bool & use_intra_process_comms() const;
  bool & use_intra_process_comms();

  /// Return a reference to the start_parameter_services flag
  /**
   * True to setup ROS interfaces for accessing parameters in the node.
   */
  const bool & start_parameter_services() const;
  bool & start_parameter_services();

protected:
  /// Clean up internal rcl_node_options_t structure.
  void
  finalize_node_options();

  /// Retrieve the ROS_DOMAIN_ID environment variable and populate options.
  void
  set_domain_id_from_env();

private:
  std::unique_ptr<rcl_node_options_t> node_options_;

  std::vector<rclcpp::Parameter> initial_parameters_;

  /// True to use the optimized intra-process communication pipeline
  bool use_intra_process_comms_ {false};

  /// True to setup ROS interfaces for accessing parameters in the node.
  bool start_parameter_services_ {true};
};

}  // namespace rclcpp

#endif  // RCLCPP__NODE_OPTIONS_HPP_
