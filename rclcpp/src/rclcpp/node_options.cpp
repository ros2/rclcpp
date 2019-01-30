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

#include "rclcpp/node_options.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

namespace rclcpp
{

NodeOptions::NodeOptions(
  rclcpp::Context::SharedPtr context,
  const std::vector<std::string> & arguments,
  const std::vector<rclcpp::Parameter> & initial_parameters,
  bool use_global_arguments,
  bool use_intra_process_comms,
  bool start_parameter_services,
  rcl_allocator_t allocator)
: node_options_(new rcl_node_options_t),
  context_(context),
  initial_parameters_(initial_parameters),
  use_intra_process_comms_(use_intra_process_comms),
  start_parameter_services_(start_parameter_services)
{
  *node_options_ = rcl_node_get_default_options();
  node_options_->allocator = allocator;
  node_options_->use_global_arguments = use_global_arguments;
  node_options_->domain_id = get_domain_id_from_env();

  std::unique_ptr<const char *[]> c_args;
  if (!arguments.empty()) {
    c_args.reset(new const char *[arguments.size()]);
    for (std::size_t i = 0; i < arguments.size(); ++i) {
      c_args[i] = arguments[i].c_str();
    }
  }

  if (arguments.size() > std::numeric_limits<int>::max()) {
    throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "Too many args");
  }

  // TODO(sloretz) Pass an allocator to argument parsing
  rmw_ret_t ret = rcl_parse_arguments(
    static_cast<int>(arguments.size()), c_args.get(), allocator,
    &(this->node_options_->arguments));

  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "failed to parse arguments");
  }
}

NodeOptions::NodeOptions(const NodeOptions & other)
: node_options_(new rcl_node_options_t)
{
  rcl_ret_t ret = rcl_node_options_copy(other.get_rcl_node_options(), node_options_.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl node options");
  }

  this->initial_parameters_ = other.initial_parameters();
  this->use_intra_process_comms_ = other.use_intra_process_comms();
  this->start_parameter_services_ = other.start_parameter_services();
}

NodeOptions &
NodeOptions::operator=(const NodeOptions & other)
{
  if (this != &other) {
    this->finalize_node_options();
    rcl_ret_t ret = rcl_node_options_copy(other.get_rcl_node_options(), node_options_.get());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl node options");
    }

    this->initial_parameters_ = other.initial_parameters();
    this->use_intra_process_comms_ = other.use_intra_process_comms();
    this->start_parameter_services_ = other.start_parameter_services();
  }
  return *this;
}

NodeOptions::~NodeOptions()
{
  this->finalize_node_options();
}

const rcl_node_options_t *
NodeOptions::get_rcl_node_options() const
{
  return this->node_options_.get();
}

rclcpp::Context::SharedPtr
NodeOptions::context() const
{
  return this->context_;
}

const std::vector<rclcpp::Parameter> &
NodeOptions::initial_parameters() const
{
  return this->initial_parameters_;
}

const bool &
NodeOptions::use_global_arguments() const
{
  return this->node_options_->use_global_arguments;
}

const bool &
NodeOptions::use_intra_process_comms() const
{
  return this->use_intra_process_comms_;
}

const bool &
NodeOptions::start_parameter_services() const
{
  return this->start_parameter_services_;
}

size_t
NodeOptions::get_domain_id_from_env() const
{
  // Determine the domain id based on the options and the ROS_DOMAIN_ID env variable.
  size_t domain_id = std::numeric_limits<size_t>::max();
  char * ros_domain_id = nullptr;
  const char * env_var = "ROS_DOMAIN_ID";
#ifndef _WIN32
  ros_domain_id = getenv(env_var);
#else
  size_t ros_domain_id_size;
  _dupenv_s(&ros_domain_id, &ros_domain_id_size, env_var);
#endif
  if (ros_domain_id) {
    uint32_t number = strtoul(ros_domain_id, NULL, 0);
    if (number == (std::numeric_limits<uint32_t>::max)()) {
#ifdef _WIN32
      // free the ros_domain_id before throwing, if getenv was used on Windows
      free(ros_domain_id);
#endif
      throw std::runtime_error("failed to interpret ROS_DOMAIN_ID as integral number");
    }
    domain_id = static_cast<size_t>(number);
#ifdef _WIN32
    free(ros_domain_id);
#endif
  }
  return domain_id;
}

void
NodeOptions::finalize_node_options()
{
  if (node_options_) {
    rcl_ret_t ret = rcl_node_options_fini(node_options_.get());
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "failed to finalize rcl node options: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
  }
}

NodeOptions
NodeOptions::Builder::build()
{
  return NodeOptions(
    context_,
    arguments_,
    initial_parameters_,
    use_global_arguments_,
    use_intra_process_comms_,
    start_parameter_services_,
    allocator_);
}

NodeOptions::Builder &
NodeOptions::Builder::context(rclcpp::Context::SharedPtr context)
{
  this->context_ = context;
  return *this;
}

NodeOptions::Builder &
NodeOptions::Builder::use_global_arguments(bool use_global_arguments)
{
  this->use_global_arguments_ = use_global_arguments;
  return *this;
}

NodeOptions::Builder &
NodeOptions::Builder::use_intra_process_comms(bool use_intra_process_comms)
{
  this->use_intra_process_comms_ = use_intra_process_comms;
  return *this;
}

NodeOptions::Builder &
NodeOptions::Builder::start_parameter_services(bool start_parameter_services)
{
  this->start_parameter_services_ = start_parameter_services;
  return *this;
}

NodeOptions::Builder &
NodeOptions::Builder::arguments(const std::vector<std::string> & arguments)
{
  this->arguments_ = arguments;
  return *this;
}

NodeOptions::Builder &
NodeOptions::Builder::initial_parameters(
  const std::vector<rclcpp::Parameter> initial_parameters)
{
  this->initial_parameters_ = initial_parameters;
  return *this;
}

NodeOptions::Builder &
NodeOptions::Builder::allocator(rcl_allocator_t allocator)
{
  this->allocator_ = allocator;
  return *this;
}


}  // namespace rclcpp
