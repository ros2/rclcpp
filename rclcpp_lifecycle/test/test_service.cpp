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

#include <chrono>
#include <string>
#include <memory>
#include <utility>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rmw/qos_profiles.h"

#include "test_msgs/srv/empty.hpp"

using namespace std::chrono_literals;

class TestService : public ::testing::Test
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
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_lifecycle_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
};

/*
   Testing service construction and destruction.
 */
TEST_F(TestService, construction_and_destruction) {
  using test_msgs::srv::Empty;
  auto callback =
    [](const Empty::Request::SharedPtr, Empty::Response::SharedPtr) {
    };
  {
    auto service = node->create_service<Empty>("service", callback);
    EXPECT_NE(nullptr, service->get_service_handle());
    const rclcpp::ServiceBase * const_service_base = service.get();
    EXPECT_NE(nullptr, const_service_base->get_service_handle());
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

    auto service = node->create_service<Empty>(
      "service", callback, rmw_qos_profile_services_default);

    // remove warning suppression
    #if !defined(_WIN32)
    # pragma GCC diagnostic pop
    #else  // !defined(_WIN32)
    # pragma warning(pop)
    #endif

    EXPECT_NE(nullptr, service->get_service_handle());
    const rclcpp::ServiceBase * const_service_base = service.get();
    EXPECT_NE(nullptr, const_service_base->get_service_handle());
  }

  {
    ASSERT_THROW(
    {
      auto service = node->create_service<Empty>("invalid_service?", callback);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}
