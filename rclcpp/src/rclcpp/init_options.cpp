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

#include "rclcpp/init_options.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp
{

InitOptions::InitOptions(rcl_allocator_t allocator)
: init_options_(new rcl_init_options_t)
{
  *init_options_ = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(init_options_.get(), allocator);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialized rcl init options");
  }
}

InitOptions::InitOptions(const rcl_init_options_t & init_options)
: init_options_(new rcl_init_options_t)
{
  *init_options_ = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_copy(&init_options, init_options_.get());
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl init options");
  }
}

InitOptions::InitOptions(const InitOptions & other)
: InitOptions(*other.get_rcl_init_options())
{
  shutdown_on_sigint = other.shutdown_on_sigint;
}

bool
InitOptions::auto_initialize_logging() const
{
  return initialize_logging_;
}

InitOptions &
InitOptions::auto_initialize_logging(bool initialize_logging)
{
  initialize_logging_ = initialize_logging;
  return *this;
}

InitOptions &
InitOptions::operator=(const InitOptions & other)
{
  if (this != &other) {
    this->finalize_init_options();
    rcl_ret_t ret = rcl_init_options_copy(other.get_rcl_init_options(), init_options_.get());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl init options");
    }
    this->shutdown_on_sigint = other.shutdown_on_sigint;
  }
  return *this;
}

InitOptions::~InitOptions()
{
  this->finalize_init_options();
}

void
InitOptions::finalize_init_options()
{
  if (init_options_) {
    rcl_ret_t ret = rcl_init_options_fini(init_options_.get());
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "failed to finalize rcl init options: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
    *init_options_ = rcl_get_zero_initialized_init_options();
  }
}

const rcl_init_options_t *
InitOptions::get_rcl_init_options() const
{
  return init_options_.get();
}

bool
InitOptions::use_default_domain_id()
{
  // Try to get the ROS_DOMAIN_ID environment variable.
  const char * ros_domain_id = NULL;
  const char * env_var = "ROS_DOMAIN_ID";
  const char * get_env_error_str = NULL;
  get_env_error_str = rcutils_get_env(env_var, &ros_domain_id);
  if (NULL != get_env_error_str) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"),
      "failed to get env var %s with %s", env_var, get_env_error_str);
    return false;
  }
  if (ros_domain_id) {
    unsigned long number = strtoul(ros_domain_id, NULL, 0);
    if (number == (std::numeric_limits<uint32_t>::max)()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "failed to interpret %s as integral number", env_var);
      return false;
    }
    init_options_->domain_id = (size_t) number;
  }

  return true;
}

void
InitOptions::set_domain_id(size_t domain_id)
{
  init_options_->domain_id = domain_id;
}

size_t
InitOptions::get_domain_id() const
{
  return init_options_->domain_id;
}

}  // namespace rclcpp
