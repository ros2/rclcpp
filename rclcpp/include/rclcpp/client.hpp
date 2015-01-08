/* Copyright 2014 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RCLCPP_RCLCPP_CLIENT_HPP_
#define RCLCPP_RCLCPP_CLIENT_HPP_

#include <memory>

#include <ros_middleware_interface/functions.h>
#include <ros_middleware_interface/handles.h>

#include <rclcpp/macros.hpp>

namespace rclcpp
{

// Forward declaration for friend statement
namespace node {class Node;}

namespace client
{

template<typename ServiceT>
class Client
{
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(Client);

  Client(ros_middleware_interface::ClientHandle client_handle,
         std::string& service_name)
    : client_handle_(client_handle), service_name_(service_name)
  {}

  std::shared_ptr<typename ServiceT::Response>
  send_request(std::shared_ptr<typename ServiceT::Request> &req, long timeout=10)
  {
    ::ros_middleware_interface::send_request(client_handle_, req.get());

    std::shared_ptr<typename ServiceT::Response> res = std::make_shared<typename ServiceT::Response>();
    bool received = ::ros_middleware_interface::receive_response(client_handle_, res.get(), timeout);
    if(!received)
    {
      // TODO: use custom exception
      throw std::runtime_error("Timed out while waiting for response");
    }
    return res;
  }

private:
  ros_middleware_interface::ClientHandle client_handle_;
  std::string service_name_;

};

} /* namespace client */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_CLIENT_HPP_ */
