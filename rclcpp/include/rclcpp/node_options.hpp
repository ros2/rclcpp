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

  /// Return the rcl node options.
  RCLCPP_PUBLIC
  const rcl_node_options_t *
  get_rcl_node_options() const;

  RCLCPP_PUBLIC
  rclcpp::Context::SharedPtr
  context() const;

  /// Return a reference to the list of initial parameters
  /**
   * Initial paramters is a list of initial values for parameters on the node.
   */
  RCLCPP_PUBLIC
  const std::vector<rclcpp::Parameter> &
  initial_parameters() const;

  /// Return a reference to the use_global_arguments flag
  /**
   * False to prevent node from using arguments passed to the process
   */
  RCLCPP_PUBLIC
  const bool &
  use_global_arguments() const;

  /// Return a reference to the use_intra_process_comms flag
  /**
   * True to use the optimized intra-process communication pipeline to pass messages bewteen nodes
   * in the same process using shared memory.
   */
  RCLCPP_PUBLIC
  const bool &
  use_intra_process_comms() const;

  /// Return a reference to the start_parameter_services flag
  /**
   * True to setup ROS interfaces for accessing parameters in the node.
   */
  RCLCPP_PUBLIC
  const bool &
  start_parameter_services() const;

protected:
  /// Create NodeOptions, optionally specifying the allocator to use.
  /**
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
  std::unique_ptr<rcl_node_options_t> node_options_;

  rclcpp::Context::SharedPtr context_;

  std::vector<std::string> arguments_;

  std::vector<rclcpp::Parameter> initial_parameters_;

  bool use_intra_process_comms_;

  bool start_parameter_services_;
};

class NodeOptions::Builder
{
public:
  Builder() = default;

  NodeOptions build();

  Builder &
  context(rclcpp::Context::SharedPtr context);

  Builder &
  arguments(const std::vector<std::string> & arguments);

  Builder &
  initial_parameters(const std::vector<rclcpp::Parameter> initial_parameters);

  Builder &
  use_global_arguments(bool use_global_arguments);

  Builder &
  use_intra_process_comms(bool use_intra_process_comms);

  Builder &
  start_parameter_services(bool start_parameter_services);

  Builder &
  allocator(rcl_allocator_t allocator);

private:
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
