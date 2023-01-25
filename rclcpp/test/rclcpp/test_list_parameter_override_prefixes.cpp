// Copyright 2023 Intrisnic.ai
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

#include <map>
#include <string>

#include <rclcpp/list_parameter_override_prefixes.hpp>

TEST(parameter, list_parameter_override_prefixes)
{
  const std::map<std::string, rclcpp::ParameterValue> overrides = {
    {"foo", {}},
    {"foo.baz", {}},
    {"foo.bar", {}},
    {"foo.bar.baz", {}},
    {"foo.bar.baz.bing", {}},
    {"foobar.baz", {}},
    {"baz", {}}
  };

  {
    auto matches = rclcpp::detail::list_parameter_override_prefixes(
      overrides, "foo");
    EXPECT_EQ(2u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foo.baz"));
    EXPECT_NE(matches.end(), matches.find("foo.bar"));
  }
  {
    auto matches = rclcpp::detail::list_parameter_override_prefixes(
      overrides, "foo.bar");
    EXPECT_EQ(1u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foo.bar.baz"));
  }
  {
    auto matches = rclcpp::detail::list_parameter_override_prefixes(
      overrides, "foo.bar.baz");
    EXPECT_EQ(1u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foo.bar.baz.bing"));
  }
  {
    auto matches = rclcpp::detail::list_parameter_override_prefixes(
      overrides, "foo.baz");
    EXPECT_EQ(0u, matches.size());
  }
  {
    auto matches = rclcpp::detail::list_parameter_override_prefixes(
      overrides, "foobar");
    EXPECT_EQ(1u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foobar.baz"));
  }
  {
    auto matches = rclcpp::detail::list_parameter_override_prefixes(
      overrides, "dne");
    EXPECT_EQ(0u, matches.size());
  }
  {
    auto matches = rclcpp::detail::list_parameter_override_prefixes(
      overrides, "");
    EXPECT_EQ(3u, matches.size());
    EXPECT_NE(matches.end(), matches.find("foo"));
    EXPECT_NE(matches.end(), matches.find("foobar"));
    EXPECT_NE(matches.end(), matches.find("baz"));
  }
}
