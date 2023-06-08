// Copyright 2022 Open Source Robotics Foundation, Inc.
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
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcl_interfaces/srv/list_parameters.hpp"

#include "rmw/qos_profiles.h"

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
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_lifecycle_node", "/ns");
  }

  void TearDown()
  {
    node_.reset();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

/*
   Testing client construction and destruction.
 */
TEST_F(TestClient, construction_and_destruction) {
  using rcl_interfaces::srv::ListParameters;
  {
    auto client = node_->create_client<ListParameters>("service");
    EXPECT_TRUE(client);
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

    auto client = node_->create_client<ListParameters>(
      "service", rmw_qos_profile_services_default);
    EXPECT_TRUE(client);

    // remove warning suppression
    #if !defined(_WIN32)
    # pragma GCC diagnostic pop
    #else  // !defined(_WIN32)
    # pragma warning(pop)
    #endif
  }
  {
    auto client = node_->create_client<ListParameters>(
      "service", rclcpp::ServicesQoS());
    EXPECT_TRUE(client);
  }

  {
    ASSERT_THROW(
    {
      auto client = node_->create_client<ListParameters>("invalid_service?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}

TEST_F(TestClient, construction_with_free_function) {
  {
    auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
      node_->get_node_base_interface(),
      node_->get_node_graph_interface(),
      node_->get_node_services_interface(),
      "service",
      rmw_qos_profile_services_default,
      nullptr);
    EXPECT_TRUE(client);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
        node_->get_node_base_interface(),
        node_->get_node_graph_interface(),
        node_->get_node_services_interface(),
        "invalid_?service",
        rmw_qos_profile_services_default,
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
  {
    auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
      node_->get_node_base_interface(),
      node_->get_node_graph_interface(),
      node_->get_node_services_interface(),
      "service",
      rclcpp::ServicesQoS(),
      nullptr);
    EXPECT_TRUE(client);
  }
  {
    ASSERT_THROW(
    {
      auto client = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
        node_->get_node_base_interface(),
        node_->get_node_graph_interface(),
        node_->get_node_services_interface(),
        "invalid_?service",
        rclcpp::ServicesQoS(),
        nullptr);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}
