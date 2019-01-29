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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp
{

NodeOptions::NodeOptions(rcl_allocator_t allocator)
: node_options_(new rcl_node_options_t)
{
  *node_options_ = rcl_node_get_default_options();
  node_options_->allocator = allocator;
}

NodeOptions::NodeOptions(const rcl_node_options_t & node_options)
: node_options_(new rcl_node_options_t)
{
  *node_options_ = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_options_copy(&node_options, node_options_.get());
  if(RCL_RET_OK != ret) {
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

const rcl_node_options_t *
NodeOptions::get_rcl_node_options() const
{
  return this->node_options_.get();
}


}  // namespace rclcpp
