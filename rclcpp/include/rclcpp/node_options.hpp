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

#ifndef RCLCPP__NODEOPTIONS_HPP_
#define RCLCPP__NODEOPTIONS_HPP_

#include <memory>

#include "rcl/node_options.h"
#include "rclcpp/parameter.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Encapsulation of options for initializing node.
class NodeOptions
{
public:
  /// Constructor which allows you to specify the allocator used within the init options.
  RCLCPP_PUBLIC
  explicit NodeOptions(rcl_allocator_t allocator = rcl_get_default_allocator());

  /// Constructor which is initialized by an existing node_options.
  RCLCPP_PUBLIC
  explicit NodeOptions(const rcl_node_options_t & node_options);

  /// Copy constructor.
  RCLCPP_PUBLIC
  NodeOptions(const NodeOptions & other);

  /// Assignment operator.
  RCLCPP_PUBLIC
  NodeOptions &
  operator=(const NodeOptions & other);

  RCLCPP_PUBLIC
  virtual
  ~NodeOptions();

  /// Return the rcl node options.
  RCLCPP_PUBLIC
  const rcl_node_options_t *
  get_rcl_node_options() const;

  /// Return the list of initial parameters
  const std::vector<rclcpp::Parameter> & initial_parameters() const;
  std::vector<rclcpp::Parameter> & initial_parameters();

  const bool & use_global_arguments() const;
  bool & use_global_arguments();

  const bool & use_intra_process_comms() const;
  bool & use_intra_process_comms();

  const bool & start_parameter_services() const;
  bool & start_parameter_services();

protected:
  void
  finalize_node_options();

  void
  set_domain_id_from_env();

private:
  std::unique_ptr<rcl_node_options_t>  node_options_;

  std::vector<rclcpp::Parameter> initial_parameters_;

  bool use_intra_process_comms_ {false};

  bool start_parameter_services_ {true};
};

}  // namespace rclcpp

#endif  // RCLCPP__NODEOPTIONS_HPP_
