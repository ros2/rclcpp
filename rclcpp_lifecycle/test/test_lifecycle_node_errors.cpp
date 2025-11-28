// Copyright 2024 Open Source Robotics Foundation, Inc.
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
#include <stdexcept>
#include <string>

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "./mocking_utils/patch.hpp"

class TestDefaultStateMachine : public ::testing::Test
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

class EmptyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EmptyLifecycleNode(const std::string & node_name)
  : rclcpp_lifecycle::LifecycleNode(node_name)
  {}
};

// This test is split out of test_lifecycle_node.cpp for an esoteric reason.  When running on
// RedHat-based distributions (like Fedora or RHEL), the way that glibc is compiled does not
// allow mocking_utils::inject_on_return to work.  Thus the test has to patch_and_return().
// Unfortunately, this means that the resources are not actually cleaned up, and thus other tests
// may return incorrect results.  By having it in a separate process we ensure that the resources
// will at least be cleaned up by the process dying.
TEST_F(TestDefaultStateMachine, empty_initializer_rcl_errors)
{
  {
    auto patch = mocking_utils::patch_and_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_state_machine_init, RCL_RET_ERROR);
    EXPECT_THROW(
      std::make_shared<EmptyLifecycleNode>("testnode").reset(),
      std::runtime_error);
  }
  {
    auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
    auto patch = mocking_utils::patch_and_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_state_machine_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(test_node.reset());
  }
}
