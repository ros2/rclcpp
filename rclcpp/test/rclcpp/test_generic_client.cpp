// Copyright 2023 Sony Group Corporation.
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
#include <utility>

#include "rclcpp/create_generic_client.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"

#include "../mocking_utils/patch.hpp"

#include "test_msgs/srv/empty.hpp"

using namespace std::chrono_literals;

// All tests are from test_client

class TestGenericClient : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class TestGenericClientSub : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_node", "/ns");
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
   Testing client construction and destruction.
 */
TEST_F(TestGenericClient, construction_and_destruction) {
  {
    auto client = node->create_generic_client("test_service", "test_msgs/srv/Empty");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_generic_client("invalid_test_service?", "test_msgs/srv/Empty");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_generic_client("test_service", "test_msgs/srv/InvalidType");
    }, std::runtime_error);
  }
}

TEST_F(TestGenericClient, construction_with_free_function) {
  {
    auto client = rclcpp::create_generic_client(
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      "test_service",
      "test_msgs/srv/Empty",
      rclcpp::ServicesQoS(),
      nullptr);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_generic_client(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        "invalid_?test_service",
        "test_msgs/srv/Empty",
        rclcpp::ServicesQoS(),
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_generic_client(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        "test_service",
        "test_msgs/srv/InvalidType",
        rclcpp::ServicesQoS(),
        nullptr);
    }, std::runtime_error);
  }
  {
    auto client = rclcpp::create_generic_client(
      node,
      "test_service",
      "test_msgs/srv/Empty",
      rclcpp::ServicesQoS(),
      nullptr);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_generic_client(
        node,
        "invalid_?test_service",
        "test_msgs/srv/Empty",
        rclcpp::ServicesQoS(),
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_generic_client(
        node,
        "invalid_?test_service",
        "test_msgs/srv/InvalidType",
        rclcpp::ServicesQoS(),
        nullptr);
    }, std::runtime_error);
  }
}

TEST_F(TestGenericClient, construct_with_rcl_error) {
  {
    // reset() is not necessary for this exception, but handles unused return value warning
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_client_init, RCL_RET_ERROR);
    EXPECT_THROW(
      node->create_generic_client("test_service", "test_msgs/srv/Empty").reset(),
      rclcpp::exceptions::RCLError);
  }
  {
    // reset() is required for this one
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_client_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(
      node->create_generic_client("test_service", "test_msgs/srv/Empty").reset());
  }
}

TEST_F(TestGenericClient, wait_for_service) {
  const std::string service_name = "test_service";

  auto client = node->create_generic_client(service_name, "test_msgs/srv/Empty");
  EXPECT_FALSE(client->wait_for_service(std::chrono::nanoseconds(0)));
  EXPECT_FALSE(client->wait_for_service(std::chrono::milliseconds(10)));

  auto callback = [](
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr) {};

  auto service =
    node->create_service<test_msgs::srv::Empty>(service_name, std::move(callback));

  EXPECT_TRUE(client->wait_for_service(std::chrono::nanoseconds(-1)));
  EXPECT_TRUE(client->service_is_ready());
}

/*
   Testing generic client construction and destruction for subnodes.
 */
TEST_F(TestGenericClientSub, construction_and_destruction) {
  {
    auto client = subnode->create_generic_client("test_service", "test_msgs/srv/Empty");
    EXPECT_STREQ(client->get_service_name(), "/ns/test_service");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_generic_client("invalid_service?", "test_msgs/srv/Empty");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}
