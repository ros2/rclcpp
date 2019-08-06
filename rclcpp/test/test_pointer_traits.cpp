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

#include <string>
#include <memory>

#include "rclcpp/pointer_traits.hpp"

TEST(TestPointerTraits, is_pointer) {
  auto ptr = new int(13);
  const auto * const cptrc = new int(13);
  auto sptr = std::make_shared<int>(13);
  const auto csptr = std::make_shared<int>(13);
  const volatile auto cvsptr = std::make_shared<int>(13);
  const volatile auto cvsptrc = std::shared_ptr<int const>(cptrc);
  auto uptr = std::make_unique<int>(13);
  const auto cuptr = std::make_unique<int>(13);
  const volatile auto cvuptr = std::make_unique<int>(13);
  const volatile auto cvuptrc = std::make_unique<const int>(13);

  auto b_ptr = rclcpp::is_pointer<decltype(ptr)>::value;
  auto b_cptrc = rclcpp::is_pointer<decltype(cptrc)>::value;
  auto b_sptr = rclcpp::is_pointer<decltype(sptr)>::value;
  auto b_csptr = rclcpp::is_pointer<decltype(csptr)>::value;
  auto b_cvsptr = rclcpp::is_pointer<decltype(cvsptr)>::value;
  auto b_cvsptrc = rclcpp::is_pointer<decltype(cvsptrc)>::value;
  auto b_uptr = rclcpp::is_pointer<decltype(uptr)>::value;
  auto b_cuptr = rclcpp::is_pointer<decltype(cuptr)>::value;
  auto b_cvuptr = rclcpp::is_pointer<decltype(cvuptr)>::value;
  auto b_cvuptrc = rclcpp::is_pointer<decltype(cvuptrc)>::value;

  EXPECT_TRUE(b_ptr);
  EXPECT_TRUE(b_cptrc);
  EXPECT_TRUE(b_sptr);
  EXPECT_TRUE(b_csptr);
  EXPECT_TRUE(b_cvsptr);
  EXPECT_TRUE(b_cvsptrc);
  EXPECT_TRUE(b_uptr);
  EXPECT_TRUE(b_cuptr);
  EXPECT_TRUE(b_cvuptr);
  EXPECT_TRUE(b_cvuptrc);
}

struct POD
{
  int i = 0;
  bool b = false;
};

TEST(TestPointerTraits, is_no_pointer) {
  int i = 13;
  std::string s = "hello world";
  POD pod;

  auto b_i = rclcpp::is_pointer<decltype(i)>::value;
  auto b_s = rclcpp::is_pointer<decltype(s)>::value;
  auto b_pod = rclcpp::is_pointer<decltype(pod)>::value;

  EXPECT_FALSE(b_i);
  EXPECT_FALSE(b_s);
  EXPECT_FALSE(b_pod);
}
