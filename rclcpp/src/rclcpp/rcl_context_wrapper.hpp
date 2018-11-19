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

#ifndef RCLCPP__RCL_CONTEXT_WRAPPER_HPP_
#define RCLCPP__RCL_CONTEXT_WRAPPER_HPP_

#include <memory>

#include "rcl/context.h"
#include "rcl/init.h"
#include "rclcpp/exceptions.hpp"

namespace rclcpp
{

struct RclContextWrapper
{
  RclContextWrapper() : context_(nullptr) {this->reset();}

  ~RclContextWrapper()
  {
    rcl_ret_t ret = rcl_shutdown(context_.get());
    if (RCL_RET_OK != ret && RCL_RET_ALREADY_SHUTDOWN != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "failed to shutdown context");
    }
    // reset will finalize the context
    this->reset();
  }

  std::shared_ptr<rcl_context_t>
  get_context() const
  {
    return context_;
  }

  void
  reset()
  {
    context_.reset(new rcl_context_t, [](rcl_context_t * context) {
      if (nullptr == context) {
        return;
      }
      rcl_ret_t ret = rcl_context_fini(context);
      if (RCL_RET_OK != ret && RCL_RET_INVALID_ARGUMENT != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret, "failed to fini context");
      }
    });
    *context_ = rcl_get_zero_initialized_context();
  }

private:
  std::shared_ptr<rcl_context_t> context_;
};

}  // namespace rclcpp

#endif  // RCLCPP__RCL_CONTEXT_WRAPPER_HPP_
