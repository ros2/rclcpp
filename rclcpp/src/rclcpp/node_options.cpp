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

NodeOptions::NodeOptions(rcl_allocator_t allocator)
: NodeOptions({}, {}, allocator)
{
}

NodeOptions::NodeOptions(
  const std::vector<std::string> & arguments,
  const std::vector<rclcpp::Parameter> & initial_parameters,
  rcl_allocator_t allocator)
: node_options_(new rcl_node_options_t)
{
  *node_options_ = rcl_node_get_default_options();
  node_options_->allocator = allocator;
  set_domain_id_from_env();

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

  initial_parameters_ = initial_parameters;
}

NodeOptions::NodeOptions(const rcl_node_options_t & node_options)
: node_options_(new rcl_node_options_t)
{
  *node_options_ = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_options_copy(&node_options, node_options_.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl node options");
  }
}

NodeOptions::NodeOptions(const NodeOptions & other)
: NodeOptions(*other.get_rcl_node_options())
{}

NodeOptions &
NodeOptions::operator=(const NodeOptions & other)
{
  if (this != &other) {
    this->finalize_node_options();
    rcl_ret_t ret = rcl_node_options_copy(other.get_rcl_node_options(), node_options_.get());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl node options");
    }
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

const std::vector<rclcpp::Parameter> &
NodeOptions::initial_parameters() const
{
  return this->initial_parameters_;
}

std::vector<rclcpp::Parameter> &
NodeOptions::initial_parameters()
{
  return this->initial_parameters_;
}

const bool &
NodeOptions::use_global_arguments() const
{
  return this->node_options_->use_global_arguments;
}

bool &
NodeOptions::use_global_arguments()
{
  return this->node_options_->use_global_arguments;
}

const bool &
NodeOptions::use_intra_process_comms() const
{
  return this->use_intra_process_comms_;
}

bool &
NodeOptions::use_intra_process_comms()
{
  return this->use_intra_process_comms_;
}

const bool &
NodeOptions::start_parameter_services() const
{
  return this->start_parameter_services_;
}

bool &
NodeOptions::start_parameter_services()
{
  return this->start_parameter_services_;
}

void
NodeOptions::set_domain_id_from_env()
{
  // Determine the domain id based on the options and the ROS_DOMAIN_ID env variable.
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
    this->node_options_->domain_id = static_cast<size_t>(number);
#ifdef _WIN32
    free(ros_domain_id);
#endif
  }
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

}  // namespace rclcpp
