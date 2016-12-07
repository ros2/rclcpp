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

#include "rclcpp/exceptions.hpp"

using rclcpp::exceptions::throw_from_rcl_error;

using rclcpp::node_interfaces::NodeBase;

NodeBase::NodeBase(
  const std::string & node_name,
  rclcpp::context::Context::SharedPtr context)
: name_(node_name),
  context_(context),
  node_handle_(nullptr),
  default_callback_group_(nullptr),
  associated_with_executor_(false),
  notify_guard_condition_is_valid_(false)
{
  // Setup the guard condition that is notified when changes occur in the graph.
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_ret_t ret = rcl_guard_condition_init(&notify_guard_condition_, guard_condition_options);
  if (ret != RCL_RET_OK) {
    throw_from_rcl_error(ret, "failed to create interrupt guard condition");
  }

  // Setup a safe exit lamda to clean up the guard condition in case of an error here.
  auto finalize_notify_guard_condition = [this]() {
      // Finalize the interrupt guard condition.
      if (rcl_guard_condition_fini(&notify_guard_condition_) != RCL_RET_OK) {
        fprintf(stderr,
          "[rclcpp::error] failed to destroy guard condition: %s\n", rcl_get_error_string_safe());
      }
    };

  // Determine the domain id based on the options and the ROS_DOMAIN_ID env variable.
  size_t domain_id = 0;
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
      // Finalize the interrupt guard condition.
      finalize_notify_guard_condition();
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

  // Create the rcl node and store it in a shared_ptr with a custom destructor.
  rcl_node_t * rcl_node = new rcl_node_t(rcl_get_zero_initialized_node());

  rcl_node_options_t options = rcl_node_get_default_options();
  // TODO(wjwwood): pass the Allocator to the options
  options.domain_id = domain_id;
  ret = rcl_node_init(rcl_node, name_.c_str(), &options);
  if (ret != RCL_RET_OK) {
    // Finalize the interrupt guard condition.
    finalize_notify_guard_condition();

    throw_from_rcl_error(ret, "failed to initialize rcl node");
  }

  node_handle_.reset(rcl_node, [](rcl_node_t * node) -> void {
    if (rcl_node_fini(node) != RCL_RET_OK) {
      fprintf(
        stderr, "Error in destruction of rcl node handle: %s\n", rcl_get_error_string_safe());
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
      fprintf(stderr,
        "[rclcpp::error] failed to destroy guard condition: %s\n", rcl_get_error_string_safe());
    }
  }
}

const std::string &
NodeBase::get_name() const
{
  return name_;
}

rclcpp::context::Context::SharedPtr
NodeBase::get_context() const
{
  return context_;
}

rcl_node_t *
NodeBase::get_rcl_node_handle() const
{
  return node_handle_.get();
}

std::shared_ptr<rcl_node_t>
NodeBase::get_shared_rcl_node_handle() const
{
  return node_handle_;
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
NodeBase::callback_group_in_node(callback_group::CallbackGroup::SharedPtr group)
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
