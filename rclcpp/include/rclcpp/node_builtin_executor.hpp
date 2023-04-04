// Copyright 2023 Sony Group Corporation.
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

#ifndef RCLCPP__NODE_BUILTIN_EXECUTOR_HPP_
#define RCLCPP__NODE_BUILTIN_EXECUTOR_HPP_

#include <memory>

#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
class NodeBuiltinExecutor
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(NodeBuiltinExecutor)

  RCLCPP_PUBLIC
  explicit NodeBuiltinExecutor(
    node_interfaces::NodeBaseInterface::SharedPtr node_base,
    node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    node_interfaces::NodeServicesInterface::SharedPtr node_services,
    const NodeOptions & node_options);

  RCLCPP_PUBLIC
  ~NodeBuiltinExecutor() = default;

private:
  RCLCPP_DISABLE_COPY(NodeBuiltinExecutor)
  class NodeBuiltinExecutorImpl;
  std::unique_ptr<NodeBuiltinExecutorImpl, std::function<void(NodeBuiltinExecutorImpl *)>> impl_;
};

}  // namespace rclcpp

#endif  // RCLCPP__NODE_BUILTIN_EXECUTOR_HPP_
