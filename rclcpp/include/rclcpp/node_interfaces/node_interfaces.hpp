// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HPP_
#define RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HPP_

#include <memory>

#include "rclcpp/detail/template_unique.hpp"
#include "rclcpp/node_interfaces/detail/node_interfaces_helpers.hpp"

namespace rclcpp
{
namespace node_interfaces
{

/// A helper class for aggregating node interfaces.
template<typename ... InterfaceTs>
class NodeInterfaces
  : public detail::NodeInterfacesSupportCheck<
    detail::NodeInterfacesStorage<InterfaceTs ...>,
    InterfaceTs ...
  >,
  public detail::NodeInterfacesSupports<
    detail::NodeInterfacesStorage<InterfaceTs ...>,
    InterfaceTs ...
  >
{
  static_assert(
    0 != sizeof ...(InterfaceTs),
    "must provide at least one interface as a template argument");
  static_assert(
    rclcpp::detail::template_unique_v<InterfaceTs ...>,
    "must provide unique template parameters");

  using NodeInterfacesSupportsT = detail::NodeInterfacesSupports<
    detail::NodeInterfacesStorage<InterfaceTs ...>,
    InterfaceTs ...
  >;

public:
  /// Create a new NodeInterfaces object using the given node-like object's interfaces.
  /**
   * Specify which interfaces you need by passing them as template parameters.
   *
   * You may also use this constructor to create a NodeInterfaces that contains a subset of
   * another NodeInterfaces' interfaces.
   *
   * You may use any of the standard node interfaces that come with rclcpp:
   *   - rclcpp::node_interfaces::NodeBaseInterface
   *   - rclcpp::node_interfaces::NodeClockInterface
   *   - rclcpp::node_interfaces::NodeGraphInterface
   *   - rclcpp::node_interfaces::NodeLoggingInterface
   *   - rclcpp::node_interfaces::NodeParametersInterface
   *   - rclcpp::node_interfaces::NodeServicesInterface
   *   - rclcpp::node_interfaces::NodeTimeSourceInterface
   *   - rclcpp::node_interfaces::NodeTimersInterface
   *   - rclcpp::node_interfaces::NodeTopicsInterface
   *   - rclcpp::node_interfaces::NodeWaitablesInterface
   *
   * Or you use custom interfaces as long as you make a template specialization
   * of the rclcpp::node_interfaces::detail::NodeInterfacesSupport struct using
   * the RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT macro.
   * If you choose not to use the helper macro, then you can specialize the
   * template yourself, but you must:
   *   - TODO
   *
   * Usage example:
   *   - TODO
   *
   * \param[in] node Node-like object from which to get the node interfaces
   */
  template<typename NodeT>
  NodeInterfaces(NodeT & node)  // NOLINT(runtime/explicit)
  : NodeInterfacesSupportsT(node)
  {}

  /// NodeT::SharedPtr Constructor
  // NOTE(CH3): We cannot dereference the shared_ptr and pass it, because otherwise a single arg
  //            aggregate constructor call that passes a NodeInterface shared_ptr will fail
  //            since it will match this constructor instead.
  //
  //            Instead, we will make do with doing the nullptr check here and enforce that
  //            support classes implement a shared_ptr<NodeT> variant.
  template<typename NodeT>
  NodeInterfaces(std::shared_ptr<NodeT> node)  // NOLINT(runtime/explicit)
  : NodeInterfaces(node ? *node : throw std::invalid_argument("given node pointer is nullptr"))
  {}

  explicit NodeInterfaces(std::shared_ptr<InterfaceTs>... args)
  : NodeInterfacesSupportsT(args ...)
  {}
};

}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HPP_
