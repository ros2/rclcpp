//  Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CREATE_SERVICE_HPP_
#define RCLCPP__CREATE_SERVICE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"
#include "service_options.hpp"

namespace rclcpp
{
/// Create a service with a given type.
/**
 * \param[in] node_base NodeBaseInterface implementation of the node on which
 *  to create the service.
 * \param[in] node_services NodeServicesInterface implementation of the node on
 *  which to create the service.
 * \param[in] service_name The name on which the service is accessible.
 * \param[in] callback The callback to call when the service gets a request.
 * \param[in] qos Quality of service profile for the service.
 * \param[in] group Callback group to handle the reply to service calls.
 * \param[in] options A way to customize the allocator
 * \return Shared pointer to the created service.
 */
template<
  typename ServiceT,
  typename CallbackT,
  typename AllocatorT = std::allocator<void>>
typename rclcpp::Service<ServiceT, AllocatorT>::SharedPtr
create_service(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeServicesInterface> node_services,
  const std::string & service_name,
  CallbackT && callback,
  const rclcpp::QoS & qos,
  rclcpp::CallbackGroup::SharedPtr group,
  const rclcpp::ServiceOptionsWithAllocator<AllocatorT> & options =
  rclcpp::ServiceOptionsWithAllocator<AllocatorT>()
)
{
  rclcpp::AnyServiceCallback<ServiceT, AllocatorT> any_service_callback;
  any_service_callback.set(std::forward<CallbackT>(callback));

  auto serv = Service<ServiceT, AllocatorT>::make_shared(
    node_base->get_shared_rcl_node_handle(),
    service_name, any_service_callback, qos, options);

  auto serv_base_ptr = std::dynamic_pointer_cast<ServiceBase>(serv);
  node_services->add_service(serv_base_ptr, group);
  return serv;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_SERVICE_HPP_
