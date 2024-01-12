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
#include <utility>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"

#include "../mocking_utils/patch.hpp"

#include "test_msgs/srv/empty.hpp"

class TestClient : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

class TestClientSub : public ::testing::Test
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
   Testing client construction and destruction.
 */
TEST_F(TestClient, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  {
    auto client = node->create_client<ListParameters>("service");
  }
  {
    // suppress deprecated function warning
    #if !defined(_WIN32)
    # pragma GCC diagnostic push
    # pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    #else  // !defined(_WIN32)
    # pragma warning(push)
    # pragma warning(disable: 4996)
    #endif

    auto client = node->create_client<ListParameters>(
      "service", rmw_qos_profile_services_default);

    // remove warning suppression
    #if !defined(_WIN32)
    # pragma GCC diagnostic pop
    #else  // !defined(_WIN32)
    # pragma warning(pop)
    #endif
  }
  {
    auto client = node->create_client<ListParameters>(
      "service", rclcpp::ServicesQoS());
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_client<ListParameters>("invalid_service?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

TEST_F(TestClient, construction_with_free_function) {
  {
    auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      "service",
      rmw_qos_profile_services_default,
      nullptr);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        "invalid_?service",
        rmw_qos_profile_services_default,
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
  {
    auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
      node->get_node_base_interface(),
      node->get_node_graph_interface(),
      node->get_node_services_interface(),
      "service",
      rclcpp::ServicesQoS(),
      nullptr);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
        node->get_node_base_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        "invalid_?service",
        rclcpp::ServicesQoS(),
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

TEST_F(TestClient, construct_with_rcl_error) {
  {
    // reset() is not necessary for this exception, but handles unused return value warning
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_client_init, RCL_RET_ERROR);
    EXPECT_THROW(
      node->create_client<test_msgs::srv::Empty>("service").reset(),
      rclcpp::exceptions::RCLError);
  }
  {
    // reset() is required for this one
    auto mock = mocking_utils::patch_and_return("lib:rclcpp", rcl_client_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(node->create_client<test_msgs::srv::Empty>("service").reset());
  }
}

TEST_F(TestClient, wait_for_service) {
  const std::string service_name = "service";
  auto client = node->create_client<test_msgs::srv::Empty>(service_name);
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
   Testing client construction and destruction for subnodes.
 */
TEST_F(TestClientSub, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  {
    auto client = subnode->create_client<ListParameters>("service");
    EXPECT_STREQ(client->get_service_name(), "/ns/sub_ns/service");
  }

  {
    ASSERT_THROW(
    {
      auto client = node->create_client<ListParameters>("invalid_service?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}
