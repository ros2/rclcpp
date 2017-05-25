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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

/*
   Testing expand_topic_or_service_name.
 */
TEST(TestExpandTopicOrServiceName, normal) {
  using rclcpp::expand_topic_or_service_name;
  {
    ASSERT_EQ("/ns/chatter", expand_topic_or_service_name("chatter", "node", "/ns"));
  }
}

/*
   Testing exceptions of expand_topic_or_service_name.
 */
TEST(TestExpandTopicOrServiceName, exceptions) {
  using rclcpp::expand_topic_or_service_name;
  {
    ASSERT_THROW({
      expand_topic_or_service_name("chatter", "invalid_node?", "/ns");
    }, rclcpp::exceptions::InvalidNodeNameError);
  }

  {
    ASSERT_THROW({
      expand_topic_or_service_name("chatter", "node", "/invalid_ns?");
    }, rclcpp::exceptions::InvalidNamespaceError);
  }

  {
    ASSERT_THROW({
      expand_topic_or_service_name("chatter/42invalid", "node", "/ns");
    }, rclcpp::exceptions::InvalidTopicNameError);
  }

  {
    ASSERT_THROW({
      // this one will only fail on the "full" topic name validation check
      expand_topic_or_service_name("chatter/{ns}/invalid", "node", "/ns");
    }, rclcpp::exceptions::InvalidTopicNameError);
  }

  {
    ASSERT_THROW({
      // is_service = true
      expand_topic_or_service_name("chatter/42invalid", "node", "/ns", true);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }

  {
    ASSERT_THROW({
      // is_service = true
      // this one will only fail on the "full" topic name validation check
      expand_topic_or_service_name("chatter/{ns}/invalid", "node", "/ns", true);
    }, rclcpp::exceptions::InvalidServiceNameError);
  }
}
