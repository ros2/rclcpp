// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <utility>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/node_interfaces/node_base_interface_traits.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp/node.hpp"

class MyNode
{
public:
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface() const
  {
    rclcpp::NodeOptions options;
    return std::make_shared<rclcpp::node_interfaces::NodeBase>(
      "my_node_name",
      "my_node_namespace",
      rclcpp::contexts::get_global_default_context(),
      *options.get_rcl_node_options(),
      options.use_intra_process_comms(),
      options.enable_topic_statistics());
  }
};

class WrongNode
{
public:
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> not_get_node_base_interface()
  {
    return nullptr;
  }
};

template<class T, typename std::enable_if<
    rclcpp::node_interfaces::has_node_base_interface<T>::value
  >::type * = nullptr>
void get_node_name(const T & nodelike)
{
  ASSERT_STREQ("my_node_name", nodelike.get_node_base_interface()->get_name());
}

class TestInterfaceTraits : public ::testing::Test
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
};

TEST_F(TestInterfaceTraits, has_node_base_interface) {
  ASSERT_TRUE(rclcpp::node_interfaces::has_node_base_interface<MyNode>::value);
  ASSERT_FALSE(rclcpp::node_interfaces::has_node_base_interface<WrongNode>::value);
  ASSERT_TRUE(rclcpp::node_interfaces::has_node_base_interface<rclcpp::Node>::value);

  get_node_name(MyNode());
}
