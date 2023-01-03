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

#ifndef RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "rcl/node.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcpputils/mutex.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// Implementation of the NodeBase part of the Node API.
class NodeBase : public NodeBaseInterface, public std::enable_shared_from_this<NodeBase>
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(NodeBase)

  /// Constructor.
  /**
   * If nullptr (default) is given for the default_callback_group, one will
   * be created by the constructor using the create_callback_group() method,
   * but virtual dispatch will not occur so overrides of that method will not
   * be used.
   */
  RCLCPP_PUBLIC
  NodeBase(
    const std::string & node_name,
    const std::string & namespace_,
    rclcpp::Context::SharedPtr context,
    const rcl_node_options_t & rcl_node_options,
    bool use_intra_process_default,
    bool enable_topic_statistics_default,
    rclcpp::CallbackGroup::SharedPtr default_callback_group = nullptr);

  RCLCPP_PUBLIC
  virtual
  ~NodeBase();

  RCLCPP_PUBLIC
  const char *
  get_name() const override;

  RCLCPP_PUBLIC
  const char *
  get_namespace() const override;

  RCLCPP_PUBLIC
  const char *
  get_fully_qualified_name() const override;

  RCLCPP_PUBLIC
  rclcpp::Context::SharedPtr
  get_context() override;

  RCLCPP_PUBLIC
  rcl_node_t *
  get_rcl_node_handle() override;

  RCLCPP_PUBLIC
  const rcl_node_t *
  get_rcl_node_handle() const override;

  RCLCPP_PUBLIC
  std::shared_ptr<rcl_node_t>
  get_shared_rcl_node_handle() override;

  RCLCPP_PUBLIC
  std::shared_ptr<const rcl_node_t>
  get_shared_rcl_node_handle() const override;

  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr
  create_callback_group(
    rclcpp::CallbackGroupType group_type,
    bool automatically_add_to_executor_with_node = true) override;

  RCLCPP_PUBLIC
  rclcpp::CallbackGroup::SharedPtr
  get_default_callback_group() override;

  RCLCPP_PUBLIC
  bool
  callback_group_in_node(rclcpp::CallbackGroup::SharedPtr group) override;

  /// Iterate over the stored callback groups, calling the given function on each valid one.
  /**
   * This method is called in a thread-safe way, and also makes sure to only call the given
   * function on those items that are still valid.
   *
   * \param[in] func The callback function to call on each valid callback group.
   */
  RCLCPP_PUBLIC
  void
  for_each_callback_group(const CallbackGroupFunction & func) override;

  RCLCPP_PUBLIC
  std::atomic_bool &
  get_associated_with_executor_atomic() override;

  RCLCPP_PUBLIC
  rclcpp::GuardCondition &
  get_notify_guard_condition() override;

  RCLCPP_PUBLIC
  bool
  get_use_intra_process_default() const override;

  bool
  get_enable_topic_statistics_default() const override;

  std::string
  resolve_topic_or_service_name(
    const std::string & name, bool is_service, bool only_expand = false) const override;

private:
  RCLCPP_DISABLE_COPY(NodeBase)

  rclcpp::Context::SharedPtr context_;
  bool use_intra_process_default_;
  bool enable_topic_statistics_default_;

  std::shared_ptr<rcl_node_t> node_handle_;

  rclcpp::CallbackGroup::SharedPtr default_callback_group_;
  rcpputils::PIMutex callback_groups_mutex_;
  std::vector<rclcpp::CallbackGroup::WeakPtr> callback_groups_;

  std::atomic_bool associated_with_executor_;

  /// Guard condition for notifying the Executor of changes to this node.
  mutable rcpputils::RecursivePIMutex notify_guard_condition_mutex_;
  rclcpp::GuardCondition notify_guard_condition_;
  bool notify_guard_condition_is_valid_;
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_BASE_HPP_
