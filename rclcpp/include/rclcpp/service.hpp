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

#ifndef RCLCPP_RCLCPP_SERVICE_HPP_
#define RCLCPP_RCLCPP_SERVICE_HPP_

#include <memory>

#include <ros_middleware_interface/functions.h>
#include <ros_middleware_interface/handles.h>

#include <rclcpp/macros.hpp>

#include <userland_msgs/RequestId.h>

namespace rclcpp
{

// Forward declaration for friend statement
namespace node {class Node;}

namespace service
{

class ServiceBase
{
  friend class rclcpp::executor::Executor;
public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(ServiceBase);

  ServiceBase(
    ros_middleware_interface::ServiceHandle service_handle,
    std::string &service_name)
    : service_handle_(service_handle), service_name_(service_name)
  {}

  std::string get_service_name()
  {
    return this->service_name_;
  }

  ros_middleware_interface::ServiceHandle get_service_handle()
  {
    return this->service_handle_;
  }

  virtual std::shared_ptr<void> create_request() = 0;
  virtual std::shared_ptr<void> create_request_header() = 0;
  virtual void handle_request(std::shared_ptr<void> &request, std::shared_ptr<void> &req_id) = 0;

private:
  RCLCPP_DISABLE_COPY(ServiceBase);

  ros_middleware_interface::ServiceHandle service_handle_;
  std::string service_name_;

};

template<typename ServiceT>
class Service : public ServiceBase
{
public:
  typedef std::function<
    void(const std::shared_ptr<typename ServiceT::Request> &,
         const std::shared_ptr<userland_msgs::RequestId> &,
         std::shared_ptr<typename ServiceT::Response>&)> CallbackType;
  RCLCPP_MAKE_SHARED_DEFINITIONS(Service);

  Service(
    ros_middleware_interface::ServiceHandle service_handle,
    std::string &service_name,
    CallbackType callback)
    : ServiceBase(service_handle, service_name), callback_(callback)
  {}

  std::shared_ptr<void> create_request()
  {
    return std::shared_ptr<void>(new typename ServiceT::Request());
  }

  std::shared_ptr<void> create_request_header()
  {
    return std::shared_ptr<void>(new userland_msgs::RequestId());
  }

  void handle_request(std::shared_ptr<void> &request, std::shared_ptr<void> &req_id)
  {
    auto typed_request = std::static_pointer_cast<typename ServiceT::Request>(request);
    auto typed_req_id = std::static_pointer_cast<userland_msgs::RequestId>(req_id);
    auto response = std::shared_ptr<typename ServiceT::Response>(new typename ServiceT::Response);
    callback_(typed_request, typed_req_id, response);
    send_response(typed_req_id, response);
  }

  void send_response(
    std::shared_ptr<userland_msgs::RequestId> &req_id,
    std::shared_ptr<typename ServiceT::Response> &response)
  {
    ::ros_middleware_interface::send_response(get_service_handle(), req_id.get(), response.get());
  }

private:
  RCLCPP_DISABLE_COPY(Service);

  CallbackType callback_;
};

} /* namespace service */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_SERVICE_HPP_ */
