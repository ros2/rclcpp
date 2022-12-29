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

#include "rclcpp/detail/template_contains.hpp"
#include "rclcpp/detail/template_unique.hpp"


TEST(NoOpTests, test_node_interfaces_template_utils) {
}                                                        // This is just to let gtest work

namespace rclcpp
{
namespace detail
{

// This tests template logic at compile time
namespace
{

struct test_template_unique
{
  static_assert(template_unique_v<int>, "failed");
  static_assert(template_unique_v<int, double>, "failed");
  static_assert(!template_unique_v<int, int>, "failed");
  static_assert(!template_unique_v<int, double, int>, "failed");
  static_assert(!template_unique_v<int, int, double>, "failed");
};


struct test_template_contains
{
  static_assert(template_contains_v<int, int, double>, "failed");
  static_assert(template_contains_v<double, int, double>, "failed");
  static_assert(template_contains_v<int, int>, "failed");
  static_assert(!template_contains_v<int>, "failed");
  static_assert(!template_contains_v<int, float>, "failed");
  static_assert(!template_contains_v<int, float, double>, "failed");
};

}   // namespace

}  // namespace detail
}  // namespace rclcpp
