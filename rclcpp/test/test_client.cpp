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

class TestClient : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::node::Node>("my_node", "/ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::node::Node::SharedPtr node;
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
    ASSERT_THROW({
      auto client = node->create_client<ListParameters>("invalid_service?");
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}
