// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CLIENT_FACTORY_HPP_
#define RCLCPP__CLIENT_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rcl/client.h"

#include "rclcpp/client.hpp"
#include "rclcpp/client_options.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/qos.hpp"

namespace rclcpp
{

/// Factory with functions used to create a specific ServiceT.
/**
 * This factory class is used to encapsulate the template generated functions
 * which are used during the creation of a specific client
 * within a non-templated class.
 *
 * It is created using the create_client_factory function, which is usually
 * called from a templated "create_client" method on the Node class, and
 * is passed to the non-templated "create_client" method on the NodeTopics
 * class where it is used to create and setup the Client.
 *
 * It also handles the single step construction of Clients, first calling
 * the constructor
 */
struct ClientFactory
{
  // Creates a ClientT<ServiceT ...> object and returns it as a ClientBase.
  using ClientFactoryFunction = std::function<
    rclcpp::ClientBase::SharedPtr(
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph)>;

  const ClientFactoryFunction create_typed_client;
};

/// Return a ClientFactory with functions setup for creating a ClientT<ServiceT, AllocatorT>.
template<typename ServiceT, typename AllocatorT, typename ClientT>
ClientFactory
create_client_factory(const rclcpp::ClientOptionsWithAllocator<AllocatorT> & options)
{
  ClientFactory factory {
    // factory function that creates a ServiceT specific ClientT
    [options](
      rclcpp::node_interfaces::NodeBaseInterface * node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      const std::string & service_name,
      const rclcpp::QoS & qos
    ) -> std::shared_ptr<ClientT>
    {
      auto client = std::make_shared<ClientT>(
        node_base, node_graph, service_name, qos, options);
      return client;
    }
  };

  // return the factory now that it is populated
  return factory;
}

}  // namespace rclcpp

#endif  // RCLCPP__CLIENT_FACTORY_HPP_
