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
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/qos.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

namespace rclcpp
{

namespace detail
{
static
void
rcl_node_options_t_destructor(rcl_node_options_t * node_options)
{
  if (node_options) {
    rcl_ret_t ret = rcl_node_options_fini(node_options);
    if (RCL_RET_OK != ret) {
      // Cannot throw here, as it may be called in the destructor.
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "failed to finalize rcl node options: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }

    delete node_options;
    node_options = nullptr;
  }
}
}  // namespace detail

NodeOptions::NodeOptions(rcl_allocator_t allocator)
: node_options_(nullptr, detail::rcl_node_options_t_destructor), allocator_(allocator)
{}

NodeOptions::NodeOptions(const NodeOptions & other)
: node_options_(nullptr, detail::rcl_node_options_t_destructor)
{
  *this = other;
}

NodeOptions &
NodeOptions::operator=(const NodeOptions & other)
{
  if (this != &other) {
    this->node_options_.reset();
    this->context_ = other.context_;
    this->arguments_ = other.arguments_;
    this->parameter_overrides_ = other.parameter_overrides_;
    this->use_global_arguments_ = other.use_global_arguments_;
    this->use_intra_process_comms_ = other.use_intra_process_comms_;
    this->start_parameter_services_ = other.start_parameter_services_;
    this->start_parameter_event_publisher_ = other.start_parameter_event_publisher_;
    this->parameter_event_qos_ = other.parameter_event_qos_;
    this->parameter_event_publisher_options_ = other.parameter_event_publisher_options_;
    this->allow_undeclared_parameters_ = other.allow_undeclared_parameters_;
    this->automatically_declare_parameters_from_overrides_ =
      other.automatically_declare_parameters_from_overrides_;
    this->allocator_ = other.allocator_;
  }
  return *this;
}

const rcl_node_options_t *
NodeOptions::get_rcl_node_options() const
{
  // If it is nullptr, create it on demand.
  if (!node_options_) {
    node_options_.reset(new rcl_node_options_t);
    *node_options_ = rcl_node_get_default_options();
    node_options_->allocator = this->allocator_;
    node_options_->use_global_arguments = this->use_global_arguments_;
    node_options_->domain_id = this->get_domain_id_from_env();

    std::unique_ptr<const char *[]> c_args;
    if (!this->arguments_.empty()) {
      c_args.reset(new const char *[this->arguments_.size()]);
      for (std::size_t i = 0; i < this->arguments_.size(); ++i) {
        c_args[i] = this->arguments_[i].c_str();
      }
    }

    if (this->arguments_.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
      throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "Too many args");
    }

    rmw_ret_t ret = rcl_parse_arguments(
      static_cast<int>(this->arguments_.size()), c_args.get(), this->allocator_,
      &(node_options_->arguments));

    if (RCL_RET_OK != ret) {
      throw_from_rcl_error(ret, "failed to parse arguments");
    }
  }

  return node_options_.get();
}

rclcpp::Context::SharedPtr
NodeOptions::context() const
{
  return this->context_;
}

NodeOptions &
NodeOptions::context(rclcpp::Context::SharedPtr context)
{
  this->context_ = context;
  return *this;
}

const std::vector<std::string> &
NodeOptions::arguments() const
{
  return this->arguments_;
}

NodeOptions &
NodeOptions::arguments(const std::vector<std::string> & arguments)
{
  this->node_options_.reset();  // reset node options to make it be recreated on next access.
  this->arguments_ = arguments;
  return *this;
}

std::vector<rclcpp::Parameter> &
NodeOptions::parameter_overrides()
{
  return this->parameter_overrides_;
}

const std::vector<rclcpp::Parameter> &
NodeOptions::parameter_overrides() const
{
  return this->parameter_overrides_;
}

NodeOptions &
NodeOptions::parameter_overrides(const std::vector<rclcpp::Parameter> & parameter_overrides)
{
  this->parameter_overrides_ = parameter_overrides;
  return *this;
}

bool
NodeOptions::use_global_arguments() const
{
  return this->use_global_arguments_;
}

NodeOptions &
NodeOptions::use_global_arguments(bool use_global_arguments)
{
  this->node_options_.reset();  // reset node options to make it be recreated on next access.
  this->use_global_arguments_ = use_global_arguments;
  return *this;
}

bool
NodeOptions::use_intra_process_comms() const
{
  return this->use_intra_process_comms_;
}

NodeOptions &
NodeOptions::use_intra_process_comms(bool use_intra_process_comms)
{
  this->use_intra_process_comms_ = use_intra_process_comms;
  return *this;
}

bool
NodeOptions::start_parameter_services() const
{
  return this->start_parameter_services_;
}

NodeOptions &
NodeOptions::start_parameter_services(bool start_parameter_services)
{
  this->start_parameter_services_ = start_parameter_services;
  return *this;
}

bool
NodeOptions::start_parameter_event_publisher() const
{
  return this->start_parameter_event_publisher_;
}

NodeOptions &
NodeOptions::start_parameter_event_publisher(bool start_parameter_event_publisher)
{
  this->start_parameter_event_publisher_ = start_parameter_event_publisher;
  return *this;
}

const rclcpp::QoS &
NodeOptions::parameter_event_qos() const
{
  return this->parameter_event_qos_;
}

NodeOptions &
NodeOptions::parameter_event_qos(const rclcpp::QoS & parameter_event_qos)
{
  this->parameter_event_qos_ = parameter_event_qos;
  return *this;
}

const rclcpp::PublisherOptionsBase &
NodeOptions::parameter_event_publisher_options() const
{
  return parameter_event_publisher_options_;
}

NodeOptions &
NodeOptions::parameter_event_publisher_options(
  const rclcpp::PublisherOptionsBase & parameter_event_publisher_options)
{
  parameter_event_publisher_options_ = parameter_event_publisher_options;
  return *this;
}

bool
NodeOptions::allow_undeclared_parameters() const
{
  return this->allow_undeclared_parameters_;
}

NodeOptions &
NodeOptions::allow_undeclared_parameters(bool allow_undeclared_parameters)
{
  this->allow_undeclared_parameters_ = allow_undeclared_parameters;
  return *this;
}

bool
NodeOptions::automatically_declare_parameters_from_overrides() const
{
  return this->automatically_declare_parameters_from_overrides_;
}

NodeOptions &
NodeOptions::automatically_declare_parameters_from_overrides(
  bool automatically_declare_parameters_from_overrides)
{
  this->automatically_declare_parameters_from_overrides_ =
    automatically_declare_parameters_from_overrides;
  return *this;
}

const rcl_allocator_t &
NodeOptions::allocator() const
{
  return this->allocator_;
}

NodeOptions &
NodeOptions::allocator(rcl_allocator_t allocator)
{
  this->node_options_.reset();  // reset node options to make it be recreated on next access.
  this->allocator_ = allocator;
  return *this;
}

// TODO(wjwwood): reuse rcutils_get_env() to avoid code duplication.
//   See also: https://github.com/ros2/rcl/issues/119
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
    uint32_t number = static_cast<uint32_t>(strtoul(ros_domain_id, NULL, 0));
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

}  // namespace rclcpp
