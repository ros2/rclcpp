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

#define ALL_RCLCPP_NODE_INTERFACES \
  rclcpp::node_interfaces::NodeBaseInterface, \
  rclcpp::node_interfaces::NodeClockInterface, \
  rclcpp::node_interfaces::NodeGraphInterface, \
  rclcpp::node_interfaces::NodeLoggingInterface, \
  rclcpp::node_interfaces::NodeParametersInterface, \
  rclcpp::node_interfaces::NodeServicesInterface, \
  rclcpp::node_interfaces::NodeTimeSourceInterface, \
  rclcpp::node_interfaces::NodeTimersInterface, \
  rclcpp::node_interfaces::NodeTopicsInterface, \
  rclcpp::node_interfaces::NodeWaitablesInterface


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
   * This allows you to aggregate interfaces from different sources together to pass as a single
   * aggregate object to any functions that take node interfaces or node-likes, without needing to
   * templatize that function.
   *
   * You may also use this constructor to create a NodeInterfaces that contains a subset of
   * another NodeInterfaces' interfaces.
   *
   * Finally, this class supports implicit conversion from node-like objects, allowing you to
   * directly pass a node-like to a function that takes a NodeInterfaces object.
   *
   * Usage examples:
   *   ```cpp
   *   // Suppose we have some function:
   *   void fn(NodeInterfaces<NodeBaseInterface, NodeClockInterface> interfaces);
   *
   *   // Then we can, explicitly:
   *   rclcpp::Node node("some_node");
   *   auto ni = NodeInterfaces<NodeBaseInterface, NodeClockInterface>(node);
   *   fn(ni);
   *
   *   // But also:
   *   fn(node);
   *
   *   // Subsetting a NodeInterfaces object also works!
   *   auto ni_base = NodeInterfaces<NodeBaseInterface>(ni);
   *
   *   // Or aggregate them (you could aggregate interfaces from disparate node-likes)
   *   auto ni_aggregated = NodeInterfaces<NodeBaseInterface, NodeClockInterface>(
   *     node->get_node_base_interface(),
   *     node->get_node_clock_interface()
   *   )
   *
   *   // And then to access the interfaces:
   *   // Get with get<>
   *   auto base = ni.get<NodeBaseInterface>();
   *
   *   // Or the appropriate getter
   *   auto clock = ni.get_clock_interface();
   *   ```
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
   *
   * Usage example:
   *   ```RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeBaseInterface, base)```
   *
   * If you choose not to use the helper macro, then you can specialize the
   * template yourself, but you must:
   *
   *   - Provide a template specialization of the get_from_node_like method that gets the interface
   *     from any node-like that stores the interface, using the node-like's getter
   *   - Designate the is_supported type as std::true_type using a using directive
   *   - Provide any number of getter methods to be used to obtain the interface with the
   *     NodeInterface object, noting that the getters of the storage class will apply to all
   *     supported interfaces.
   *     - The getter method names should not clash in name with any other interface getter
   *       specializations if those other interfaces are meant to be aggregated in the same
   *       NodeInterfaces object.
   *
   * \param[in] node Node-like object from which to get the node interfaces
   */
  template<typename NodeT>
  NodeInterfaces(NodeT & node)  // NOLINT(runtime/explicit)
  : NodeInterfacesSupportsT(node)
  {}

  explicit NodeInterfaces(std::shared_ptr<InterfaceTs>... args)
  : NodeInterfacesSupportsT(args ...)
  {}
};


}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__NODE_INTERFACES_HPP_
