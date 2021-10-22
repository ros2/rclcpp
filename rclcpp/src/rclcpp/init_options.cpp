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
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to initialize rcl init options");
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
  initialize_logging_ = other.initialize_logging_;
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
    std::lock_guard<std::mutex> init_options_lock(init_options_mutex_);
    this->finalize_init_options_impl();
    rcl_ret_t ret = rcl_init_options_copy(other.get_rcl_init_options(), init_options_.get());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl init options");
    }
    this->shutdown_on_sigint = other.shutdown_on_sigint;
    this->initialize_logging_ = other.initialize_logging_;
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
  std::lock_guard<std::mutex> init_options_lock(init_options_mutex_);
  this->finalize_init_options_impl();
}

void
InitOptions::finalize_init_options_impl()
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

void
InitOptions::use_default_domain_id()
{
  size_t domain_id = RCL_DEFAULT_DOMAIN_ID;
  rcl_ret_t ret = rcl_get_default_domain_id(&domain_id);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get default domain id");
  }
  set_domain_id(domain_id);
}

void
InitOptions::set_domain_id(size_t domain_id)
{
  std::lock_guard<std::mutex> init_options_lock(init_options_mutex_);
  rcl_ret_t ret = rcl_init_options_set_domain_id(init_options_.get(), domain_id);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to set domain id to rcl init options");
  }
}

size_t
InitOptions::get_domain_id() const
{
  std::lock_guard<std::mutex> init_options_lock(init_options_mutex_);
  size_t domain_id;
  rcl_ret_t ret = rcl_init_options_get_domain_id(init_options_.get(), &domain_id);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to get domain id from rcl init options");
  }

  return domain_id;
}

}  // namespace rclcpp
