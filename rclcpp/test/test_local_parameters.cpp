// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <cstdio>
#include <map>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

TEST(test_local_parameters, set_parameter_if_not_set) {
  auto node = rclcpp::Node::make_shared(
    "test_local_parameters_set_parameter_if_not_set",
    rclcpp::NodeOptions().allow_undeclared_parameters(true));

  {
    // try to set a map of parameters
    std::map<std::string, double> bar_map{
      {"x", 0.5},
      {"y", 1.0},
    };
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
    node->set_parameters_if_not_set("bar", bar_map);
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
    double bar_x_value;
    ASSERT_TRUE(node->get_parameter("bar.x", bar_x_value));
    EXPECT_EQ(bar_x_value, 0.5);
    double bar_y_value;
    ASSERT_TRUE(node->get_parameter("bar.y", bar_y_value));
    EXPECT_EQ(bar_y_value, 1.0);
    std::map<std::string, double> new_map;
    ASSERT_TRUE(node->get_parameters("bar", new_map));
    ASSERT_EQ(new_map.size(), 2U);
    EXPECT_EQ(new_map["x"], 0.5);
    EXPECT_EQ(new_map["y"], 1.0);
  }

  {
    // try to get a map of parameters that doesn't exist
    std::map<std::string, double> no_exist_map;
    ASSERT_FALSE(node->get_parameters("no_exist", no_exist_map));
  }

  {
    // set parameters for a map with different types, then try to get them back as a map
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
    node->set_parameter_if_not_set("baz.x", 1.0);
    node->set_parameter_if_not_set("baz.y", "hello");
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
    std::map<std::string, double> baz_map;
    EXPECT_THROW(node->get_parameters("baz", baz_map), rclcpp::ParameterTypeException);
  }
}

int main(int argc, char ** argv)
{
  ::setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // NOTE: use custom main to ensure that rclcpp::init is called only once
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
