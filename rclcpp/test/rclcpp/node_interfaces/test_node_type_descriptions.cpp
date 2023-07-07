// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_type_descriptions.hpp"

class TestNodeTypeDescriptions : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestNodeTypeDescriptions, interface_created)
{
  rclcpp::Node node{"node", "ns"};
  ASSERT_NE(nullptr, node.get_node_type_descriptions_interface());
}

TEST_F(TestNodeTypeDescriptions, disabled_no_service)
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("start_type_description_service", false);
  rclcpp::Node node{"node", "ns", node_options};

  rcl_node_t * rcl_node = node.get_node_base_interface()->get_rcl_node_handle();
  rcl_service_t * srv = nullptr;
  rcl_ret_t ret = rcl_node_get_type_description_service(rcl_node, &srv);
  ASSERT_EQ(RCL_RET_NOT_INIT, ret);
}

TEST_F(TestNodeTypeDescriptions, enabled_creates_service)
{
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("start_type_description_service", true);
  rclcpp::Node node{"node", "ns", node_options};

  rcl_node_t * rcl_node = node.get_node_base_interface()->get_rcl_node_handle();
  rcl_service_t * srv = nullptr;
  rcl_ret_t ret = rcl_node_get_type_description_service(rcl_node, &srv);
  ASSERT_EQ(RCL_RET_OK, ret);
  ASSERT_NE(nullptr, srv);
}
