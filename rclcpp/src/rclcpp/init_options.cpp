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
{}

InitOptions &
InitOptions::operator=(const InitOptions & other)
{
  if (this != &other) {
    this->finalize_init_options();
    rcl_ret_t ret = rcl_init_options_copy(other.get_rcl_init_options(), init_options_.get());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to copy rcl init options");
    }
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

}  // namespace rclcpp
