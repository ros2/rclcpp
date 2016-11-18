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

#include <gtest/gtest.h>

#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/any_service_callback.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl/service.h"

#include "rclcpp/srv/mock.hpp"
#include "rclcpp/srv/mock.h"

class TestExternallyDefinedServices : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

void
callback(const std::shared_ptr<rclcpp::srv::Mock::Request> /*req*/,
    std::shared_ptr<rclcpp::srv::Mock::Response> /*resp*/);

TEST_F(TestExternallyDefinedServices, extern_defined_uninitialized) {
  auto node_handle = rclcpp::node::Node::make_shared("base_node");

  // mock for externally defined service
  rcl_service_t service_handle = rcl_get_zero_initialized_service();

  rclcpp::any_service_callback::AnyServiceCallback<rclcpp::srv::Mock> cb;

  // don't initialize the service
  // expect fail
  try
  {
    rclcpp::service::Service<rclcpp::srv::Mock>(node_handle->get_shared_node_handle(), &service_handle, cb);
  } catch(const std::runtime_error & e){
    SUCCEED();
    return;
  }

  FAIL();
}

TEST_F(TestExternallyDefinedServices, extern_defined_initialized) {
  auto node_handle = rclcpp::node::Node::make_shared("base_node");

  // mock for externally defined service
  rcl_service_t service_handle = rcl_get_zero_initialized_service();
  rcl_service_options_t service_options = rcl_service_get_default_options();
  const rosidl_service_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(
      rclcpp, srv, Mock);
  if (rcl_service_init(&service_handle, node_handle->get_rcl_node_handle(),
      ts, "base_node_service", &service_options) != RCL_RET_OK)
  {
    FAIL();
    return;
  }
  rclcpp::any_service_callback::AnyServiceCallback<rclcpp::srv::Mock> cb;
  // don't initialize the service
  // expect fail
  try
  {
    rclcpp::service::Service<rclcpp::srv::Mock>(node_handle->get_shared_node_handle(), &service_handle, cb);
  } catch(const std::runtime_error & e){
    FAIL();
    return;
  }

  SUCCEED();
}

TEST_F(TestExternallyDefinedServices, extern_defined_destructor) {
  auto node_handle = rclcpp::node::Node::make_shared("base_node");

  // mock for externally defined service
  rcl_service_t service_handle = rcl_get_zero_initialized_service();
  rcl_service_options_t service_options = rcl_service_get_default_options();
  const rosidl_service_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(
      rclcpp, srv, Mock);
  if (rcl_service_init(&service_handle, node_handle->get_rcl_node_handle(),
      ts, "base_node_service", &service_options) != RCL_RET_OK)
  {
    FAIL();
    return;
  }
  rclcpp::any_service_callback::AnyServiceCallback<rclcpp::srv::Mock> cb;

  {
  rclcpp::service::Service<rclcpp::srv::Mock>(node_handle->get_shared_node_handle(), &service_handle, cb);
  }

  if (service_handle.impl == NULL)
  {
    FAIL();
    return;
  }
  SUCCEED();
}


