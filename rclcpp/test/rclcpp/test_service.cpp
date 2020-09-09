// Copyright 2017 Open Source Robotics Foundation, Inc.
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
#include <memory>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "test_msgs/srv/empty.hpp"
#include "test_msgs/srv/empty.h"

class TestService : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class TestServiceSub : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
    subnode = node->create_sub_node("sub_ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr subnode;
};

/*
   Testing service construction and destruction.
 */
TEST_F(TestService, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  auto callback =
    [](const ListParameters::Request::SharedPtr, ListParameters::Response::SharedPtr) {
    };
  {
    auto service = node->create_service<ListParameters>("service", callback);
  }

  {
    ASSERT_THROW(
    {
      auto service = node->create_service<ListParameters>("invalid_service?", callback);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

/*
   Testing service construction and destruction for subnodes.
 */
TEST_F(TestServiceSub, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  auto callback =
    [](const ListParameters::Request::SharedPtr, ListParameters::Response::SharedPtr) {
    };
  {
    auto service = subnode->create_service<ListParameters>("service", callback);
    EXPECT_STREQ(service->get_service_name(), "/ns/sub_ns/service");
  }

  {
    ASSERT_THROW(
    {
      auto service = node->create_service<ListParameters>("invalid_service?", callback);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

/* Testing basic getters */
TEST_F(TestService, basic_public_getters) {
  using rcl_interfaces::srv::ListParameters;
  auto callback =
    [](const ListParameters::Request::SharedPtr, ListParameters::Response::SharedPtr) {
    };
  auto service = node->create_service<ListParameters>("service", callback);
  EXPECT_STREQ(service->get_service_name(), "/ns/service");
  std::shared_ptr<rcl_service_t> service_handle = service->get_service_handle();
  EXPECT_NE(nullptr, service_handle);

  {
    // Create a extern defined const service
    auto node_handle_int = rclcpp::Node::make_shared("base_node");
    rcl_service_t service_handle = rcl_get_zero_initialized_service();
    rcl_service_options_t service_options = rcl_service_get_default_options();
    const rosidl_service_type_support_t * ts =
      rosidl_typesupport_cpp::get_service_type_support_handle<test_msgs::srv::Empty>();
    rcl_ret_t ret = rcl_service_init(
      &service_handle,
      node_handle_int->get_node_base_interface()->get_rcl_node_handle(),
      ts, "base_node_service", &service_options);
    if (ret != RCL_RET_OK) {
      FAIL();
      return;
    }
    rclcpp::AnyServiceCallback<test_msgs::srv::Empty> cb;
    const rclcpp::Service<test_msgs::srv::Empty> base(
      node_handle_int->get_node_base_interface()->get_shared_rcl_node_handle(),
      &service_handle, cb);
    // Use get_service_handle specific to const service
    std::shared_ptr<const rcl_service_t> const_service_handle = base.get_service_handle();
    EXPECT_NE(nullptr, const_service_handle);
  }
}
