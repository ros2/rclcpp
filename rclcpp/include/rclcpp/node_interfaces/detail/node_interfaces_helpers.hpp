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

#ifndef RCLCPP__NODE_INTERFACES__DETAIL__NODE_INTERFACES_HELPERS_HPP_
#define RCLCPP__NODE_INTERFACES__DETAIL__NODE_INTERFACES_HELPERS_HPP_

#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace node_interfaces
{
namespace detail
{

// Support and Helper template classes for the NodeInterfaces class.

template<typename NodeT, typename ... Ts>
std::tuple<std::shared_ptr<Ts>...>
init_tuple(NodeT & n);

/// Stores the interfaces in a tuple, provides constructors, and getters.
template<typename ... InterfaceTs>
struct NodeInterfacesStorage
{
  template<typename NodeT>
  NodeInterfacesStorage(NodeT & node)  // NOLINT(runtime/explicit)
  : interfaces_(init_tuple<decltype(node), InterfaceTs ...>(node))
  {}

  NodeInterfacesStorage()
  : interfaces_()
  {}

  explicit NodeInterfacesStorage(std::shared_ptr<InterfaceTs>... args)
  : interfaces_(args ...)
  {}

  /// Individual Node Interface non-const getter.
  template<typename NodeInterfaceT>
  std::shared_ptr<NodeInterfaceT>
  get()
  {
    static_assert(
      (std::is_same_v<NodeInterfaceT, InterfaceTs>|| ...),
      "NodeInterfaces class does not contain given NodeInterfaceT");
    return std::get<std::shared_ptr<NodeInterfaceT>>(interfaces_);
  }

  /// Individual Node Interface const getter.
  template<typename NodeInterfaceT>
  std::shared_ptr<const NodeInterfaceT>
  get() const
  {
    static_assert(
      (std::is_same_v<NodeInterfaceT, InterfaceTs>|| ...),
      "NodeInterfaces class does not contain given NodeInterfaceT");
    return std::get<std::shared_ptr<NodeInterfaceT>>(interfaces_);
  }

protected:
  std::tuple<std::shared_ptr<InterfaceTs>...> interfaces_;
};

/// Prototype of NodeInterfacesSupports.
/**
 * Should read NodeInterfacesSupports<..., T, ...> as "NodeInterfaces supports T", and
 * if NodeInterfacesSupport is specialized for T, the is_supported should be
 * set to std::true_type, but by default it is std::false_type, which will
 * lead to a compiler error when trying to use T with NodeInterfaces.
 */
template<typename StorageClassT, typename ... Ts>
struct NodeInterfacesSupports;

/// Prototype of NodeInterfacesSupportCheck template meta-function.
/**
 * This meta-function checks that all the types given are supported,
 * throwing a more human-readable error if an unsupported type is used.
 */
template<typename StorageClassT, typename ... InterfaceTs>
struct NodeInterfacesSupportCheck;

/// Iterating specialization that ensures classes are supported and inherited.
template<typename StorageClassT, typename NextInterfaceT, typename ... RemainingInterfaceTs>
struct NodeInterfacesSupportCheck<StorageClassT, NextInterfaceT, RemainingInterfaceTs ...>
  : public NodeInterfacesSupportCheck<StorageClassT, RemainingInterfaceTs ...>
{
  static_assert(
    NodeInterfacesSupports<StorageClassT, NextInterfaceT>::is_supported::value,
    "given NodeInterfaceT is not supported by rclcpp::node_interfaces::NodeInterfaces");
};

/// Terminating case when there are no more "RemainingInterfaceTs".
template<typename StorageClassT>
struct NodeInterfacesSupportCheck<StorageClassT>
{};

/// Default specialization, needs to be specialized for each supported interface.
template<typename StorageClassT, typename ... RemainingInterfaceTs>
struct NodeInterfacesSupports
{
  // Specializations need to set this to std::true_type in addition to other interfaces.
  using is_supported = std::false_type;
};

/// Terminating specialization of NodeInterfacesSupports.
template<typename StorageClassT>
struct NodeInterfacesSupports<StorageClassT>
  : public StorageClassT
{
  /// Perfect forwarding constructor to get arguments down to StorageClassT.
  template<typename ... ArgsT>
  explicit NodeInterfacesSupports(ArgsT && ... args)
  : StorageClassT(std::forward<ArgsT>(args) ...)
  {}
};

// Helper functions to initialize the tuple in NodeInterfaces.

template<typename StorageClassT, typename ElementT, typename TupleT, typename NodeT>
void
init_element(TupleT & t, NodeT & n)
{
  std::get<std::shared_ptr<ElementT>>(t) =
    NodeInterfacesSupports<StorageClassT, ElementT>::get_from_node_like(n);
}

template<typename NodeT, typename ... Ts>
std::tuple<std::shared_ptr<Ts>...>
init_tuple(NodeT & n)
{
  using StorageClassT = NodeInterfacesStorage<Ts ...>;
  std::tuple<std::shared_ptr<Ts>...> t;
  (init_element<StorageClassT, Ts>(t, n), ...);
  return t;
}

/// Macro for creating specializations with less boilerplate.
/**
 * You can use this macro to add support for your interface class if:
 *
 * - The standard getter is get_node_{NodeInterfaceName}_interface(), and
 * - the getter returns a non-const shared_ptr<{NodeInterfaceType}>
 *
 * Examples of using this can be seen in the standard node interface headers
 * in rclcpp, e.g. rclcpp/node_interfaces/node_base_interface.hpp has:
 *
 *   RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(rclcpp::node_interfaces::NodeBaseInterface, base)
 *
 * If your interface has a non-standard getter, or you want to instrument it or
 * something like that, then you'll need to create your own specialization of
 * the NodeInterfacesSupports struct without this macro.
 */
// *INDENT-OFF*
#define RCLCPP_NODE_INTERFACE_HELPERS_SUPPORT(NodeInterfaceType, NodeInterfaceName) \
  namespace rclcpp::node_interfaces::detail { \
  template<typename StorageClassT, typename ... RemainingInterfaceTs> \
  struct NodeInterfacesSupports< \
    StorageClassT, \
    NodeInterfaceType, \
    RemainingInterfaceTs ...> \
    : public NodeInterfacesSupports<StorageClassT, RemainingInterfaceTs ...> \
  { \
    using is_supported = std::true_type; \
 \
    template<typename NodeT> \
    static \
    std::shared_ptr<NodeInterfaceType> \
    get_from_node_like(NodeT & node_like) \
    { \
      return node_like.get_node_ ## NodeInterfaceName ## _interface(); \
    } \
 \
    /* Perfect forwarding constructor to get arguments down to StorageClassT (eventually). */ \
    template<typename ... ArgsT> \
    explicit NodeInterfacesSupports(ArgsT && ... args) \
    : NodeInterfacesSupports<StorageClassT, RemainingInterfaceTs ...>( \
        std::forward<ArgsT>(args) ...) \
    {} \
 \
    std::shared_ptr<NodeInterfaceType> \
    get_node_ ## NodeInterfaceName ## _interface() \
    { \
      return StorageClassT::template get<NodeInterfaceType>(); \
    } \
  }; \
  }  // namespace rclcpp::node_interfaces::detail
// *INDENT-ON*

}  // namespace detail
}  // namespace node_interfaces
}  // namespace rclcpp

#endif  // RCLCPP__NODE_INTERFACES__DETAIL__NODE_INTERFACES_HELPERS_HPP_
