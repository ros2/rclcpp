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

#ifndef RCLCPP__SERVICE_FACTORY_HPP_
#define RCLCPP__SERVICE_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rcl/service.h"

#include "rclcpp/service.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/service_options.hpp"
#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

/// Factory with functions used to create a specific ServiceT.
/**
 * This factory class is used to encapsulate the template generated functions
 * which are used during the creation of a specific service
 * within a non-templated class.
 *
 * It is created using the create_service_factory function, which is usually
 * called from a templated "create_service" method on the Node class, and
 * is passed to the non-templated "create_service" method on the NodeTopics
 * class where it is used to create and setup the Service.
 *
 * It also handles the single step construction of Services, first calling
 * the constructor
 */
struct ServiceFactory
{
  // Creates a service object and returns it as a ServiceBase.
  using ServiceFactoryFunction = std::function<
    rclcpp::ServiceBase::SharedPtr(std::shared_ptr<rcl_node_t> node_handle)>;

  const ServiceFactoryFunction create_typed_service;
};

/// Return a ServiceFactory with functions setup for creating a
/// ServiceClass<ServiceT, AllocatorT>
template<typename ServiceT, typename AllocatorT, typename ServiceClass>
ServiceFactory
create_service_factory(const rclcpp::ServiceOptionsWithAllocator<AllocatorT> & options)
{
  ServiceFactory factory {
    // factory function that creates a specific ServiceT
    [options](
      std::shared_ptr<rcl_node_t> node_handle,
      const std::string & service_name,
      AnyServiceCallback<ServiceT, AllocatorT> any_callback,
      const rclcpp::QoS & qos
    ) -> std::shared_ptr<ServiceClass>
    {
      auto service = std::make_shared<ServiceClass>(
        node_handle, service_name, any_callback, qos, options);
      return service;
    }
  };

  // return the factory now that it is populated
  return factory;
}

}  // namespace rclcpp

#endif  // RCLCPP__SERVICE_FACTORY_HPP_
