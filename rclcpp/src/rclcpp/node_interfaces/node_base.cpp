// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <string>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/node_interfaces/node_base.hpp"

#include "rcl/arguments.h"
#include "rclcpp/exceptions.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"

using rclcpp::exceptions::throw_from_rcl_error;

using rclcpp::node_interfaces::NodeBase;

NodeBase::NodeBase(
  const std::string & node_name,
  const std::string & namespace_,
  rclcpp::Context::SharedPtr context,
  const rcl_node_options_t & rcl_node_options,
  bool use_intra_process_default)
: context_(context),
  use_intra_process_default_(use_intra_process_default),
  node_handle_(nullptr),
  default_callback_group_(nullptr),
  associated_with_executor_(false),
  notify_guard_condition_is_valid_(false)
{
  // Setup the guard condition that is notified when changes occur in the graph.
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_ret_t ret = rcl_guard_condition_init(
    &notify_guard_condition_, context_->get_rcl_context().get(), guard_condition_options);
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "failed to create interrupt guard condition");
  }

  // Setup a safe exit lamda to clean up the guard condition in case of an error here.
  auto finalize_notify_guard_condition = [this]() {
      // Finalize the interrupt guard condition.
      if (rcl_guard_condition_fini(&notify_guard_condition_) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "failed to destroy guard condition: %s", rcl_get_error_string().str);
      }
    };

  // Create the rcl node and store it in a shared_ptr with a custom destructor.
  std::unique_ptr<rcl_node_t> rcl_node(new rcl_node_t(rcl_get_zero_initialized_node()));

  ret = rcl_node_init(
    rcl_node.get(),
    node_name.c_str(), namespace_.c_str(),
    context_->get_rcl_context().get(), &rcl_node_options);
  if (ret != RCL_RET_OK) {
    // Finalize the interrupt guard condition.
    finalize_notify_guard_condition();

    if (ret == RCL_RET_NODE_INVALID_NAME) {
      rcl_reset_error();  // discard rcl_node_init error
      int validation_result;
      size_t invalid_index;
      rmw_ret_t rmw_ret =
        rmw_validate_node_name(node_name.c_str(), &validation_result, &invalid_index);
      if (rmw_ret != RMW_RET_OK) {
        if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
          throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate node name");
        }
        throw_from_rcl_error(RCL_RET_ERROR, "failed to validate node name");
      }

      if (validation_result != RMW_NODE_NAME_VALID) {
        throw rclcpp::exceptions::InvalidNodeNameError(
                node_name.c_str(),
                rmw_node_name_validation_result_string(validation_result),
                invalid_index);
      } else {
        throw std::runtime_error("valid rmw node name but invalid rcl node name");
      }
    }

    if (ret == RCL_RET_NODE_INVALID_NAMESPACE) {
      rcl_reset_error();  // discard rcl_node_init error
      int validation_result;
      size_t invalid_index;
      rmw_ret_t rmw_ret =
        rmw_validate_namespace(namespace_.c_str(), &validation_result, &invalid_index);
      if (rmw_ret != RMW_RET_OK) {
        if (rmw_ret == RMW_RET_INVALID_ARGUMENT) {
          throw_from_rcl_error(RCL_RET_INVALID_ARGUMENT, "failed to validate namespace");
        }
        throw_from_rcl_error(RCL_RET_ERROR, "failed to validate namespace");
      }

      if (validation_result != RMW_NAMESPACE_VALID) {
        throw rclcpp::exceptions::InvalidNamespaceError(
                namespace_.c_str(),
                rmw_namespace_validation_result_string(validation_result),
                invalid_index);
      } else {
        throw std::runtime_error("valid rmw node namespace but invalid rcl node namespace");
      }
    }
    throw_from_rcl_error(ret, "failed to initialize rcl node");
  }

  node_handle_.reset(
    rcl_node.release(),
    [](rcl_node_t * node) -> void {
      if (rcl_node_fini(node) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "Error in destruction of rcl node handle: %s", rcl_get_error_string().str);
      }
      delete node;
    });

  // Create the default callback group.
  using rclcpp::callback_group::CallbackGroupType;
  default_callback_group_ = create_callback_group(CallbackGroupType::MutuallyExclusive);

  // Indicate the notify_guard_condition is now valid.
  notify_guard_condition_is_valid_ = true;
}

NodeBase::~NodeBase()
{
  // Finalize the interrupt guard condition after removing self from graph listener.
  {
    std::lock_guard<std::recursive_mutex> notify_condition_lock(notify_guard_condition_mutex_);
    notify_guard_condition_is_valid_ = false;
    if (rcl_guard_condition_fini(&notify_guard_condition_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "failed to destroy guard condition: %s", rcl_get_error_string().str);
    }
  }
}

const char *
NodeBase::get_name() const
{
  return rcl_node_get_name(node_handle_.get());
}

const char *
NodeBase::get_namespace() const
{
  return rcl_node_get_namespace(node_handle_.get());
}

const char *
NodeBase::get_fully_qualified_name() const
{
  return rcl_node_get_fully_qualified_name(node_handle_.get());
}

rclcpp::Context::SharedPtr
NodeBase::get_context()
{
  return context_;
}

rcl_node_t *
NodeBase::get_rcl_node_handle()
{
  return node_handle_.get();
}

const rcl_node_t *
NodeBase::get_rcl_node_handle() const
{
  return node_handle_.get();
}

std::shared_ptr<rcl_node_t>
NodeBase::get_shared_rcl_node_handle()
{
  return node_handle_;
}

std::shared_ptr<const rcl_node_t>
NodeBase::get_shared_rcl_node_handle() const
{
  return node_handle_;
}

bool
NodeBase::assert_liveliness() const
{
  return RCL_RET_OK == rcl_node_assert_liveliness(get_rcl_node_handle());
}

rclcpp::callback_group::CallbackGroup::SharedPtr
NodeBase::create_callback_group(rclcpp::callback_group::CallbackGroupType group_type)
{
  using rclcpp::callback_group::CallbackGroup;
  using rclcpp::callback_group::CallbackGroupType;
  auto group = CallbackGroup::SharedPtr(new CallbackGroup(group_type));
  callback_groups_.push_back(group);
  return group;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
NodeBase::get_default_callback_group()
{
  return default_callback_group_;
}

bool
NodeBase::callback_group_in_node(rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  bool group_belongs_to_this_node = false;
  for (auto & weak_group : this->callback_groups_) {
    auto cur_group = weak_group.lock();
    if (cur_group && (cur_group == group)) {
      group_belongs_to_this_node = true;
    }
  }
  return group_belongs_to_this_node;
}

const std::vector<rclcpp::callback_group::CallbackGroup::WeakPtr> &
NodeBase::get_callback_groups() const
{
  return callback_groups_;
}

std::atomic_bool &
NodeBase::get_associated_with_executor_atomic()
{
  return associated_with_executor_;
}

rcl_guard_condition_t *
NodeBase::get_notify_guard_condition()
{
  std::lock_guard<std::recursive_mutex> notify_condition_lock(notify_guard_condition_mutex_);
  if (!notify_guard_condition_is_valid_) {
    return nullptr;
  }
  return &notify_guard_condition_;
}

std::unique_lock<std::recursive_mutex>
NodeBase::acquire_notify_guard_condition_lock() const
{
  return std::unique_lock<std::recursive_mutex>(notify_guard_condition_mutex_);
}

bool
NodeBase::get_use_intra_process_default() const
{
  return use_intra_process_default_;
}
