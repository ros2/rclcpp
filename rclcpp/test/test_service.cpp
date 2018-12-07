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
    ASSERT_THROW({
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
    ASSERT_THROW({
      auto service = node->create_service<ListParameters>("invalid_service?", callback);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}
