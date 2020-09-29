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

#include <memory>
#include <string>
#include <vector>

#include "rcl/init.h"
#include "rcl/logging.h"

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/scope_exit.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

TEST(TestUtilities, remove_ros_arguments) {
  const char * const argv[] = {
    "process_name",
    "-d", "--ros-args",
    "-r", "__ns:=/foo/bar",
    "-r", "__ns:=/fiz/buz",
    "--", "--foo=bar", "--baz"
  };
  int argc = sizeof(argv) / sizeof(const char *);
  auto args = rclcpp::remove_ros_arguments(argc, argv);

  ASSERT_EQ(4u, args.size());
  ASSERT_EQ(std::string{"process_name"}, args[0]);
  ASSERT_EQ(std::string{"-d"}, args[1]);
  ASSERT_EQ(std::string{"--foo=bar"}, args[2]);
  ASSERT_EQ(std::string{"--baz"}, args[3]);
}

TEST(TestUtilities, init_with_args) {
  EXPECT_FALSE(rclcpp::signal_handlers_installed());
  const char * const argv[] = {"process_name"};
  int argc = sizeof(argv) / sizeof(const char *);
  auto other_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  ASSERT_EQ(1u, other_args.size());
  ASSERT_EQ(std::string{"process_name"}, other_args[0]);
  EXPECT_TRUE(rclcpp::signal_handlers_installed());

  EXPECT_TRUE(rclcpp::ok());
  rclcpp::shutdown();
}

TEST(TestUtilities, multi_init) {
  auto context1 = std::make_shared<rclcpp::contexts::DefaultContext>();
  auto context2 = std::make_shared<rclcpp::contexts::DefaultContext>();

  EXPECT_FALSE(rclcpp::ok(context1));
  EXPECT_FALSE(rclcpp::ok(context2));

  context1->init(0, nullptr);

  EXPECT_TRUE(rclcpp::ok(context1));
  EXPECT_FALSE(rclcpp::ok(context2));

  context2->init(0, nullptr);

  EXPECT_TRUE(rclcpp::ok(context1));
  EXPECT_TRUE(rclcpp::ok(context2));

  rclcpp::shutdown(context1);

  EXPECT_FALSE(rclcpp::ok(context1));
  EXPECT_TRUE(rclcpp::ok(context2));

  rclcpp::shutdown(context2);

  EXPECT_FALSE(rclcpp::ok(context1));
  EXPECT_FALSE(rclcpp::ok(context2));
}

TEST(TestUtilities, test_context_basic_access) {
  auto context1 = std::make_shared<rclcpp::contexts::DefaultContext>();
  EXPECT_NE(nullptr, context1->get_init_options().get_rcl_init_options());
  EXPECT_EQ(0u, context1->get_on_shutdown_callbacks().size());
  EXPECT_EQ(std::string{""}, context1->shutdown_reason());
}

TEST(TestUtilities, test_context_basic_access_const_methods) {
  auto context1 = std::make_shared<const rclcpp::contexts::DefaultContext>();

  EXPECT_NE(nullptr, context1->get_init_options().get_rcl_init_options());
  EXPECT_EQ(0u, context1->get_on_shutdown_callbacks().size());
}

MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, >)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcl_guard_condition_options_t, <)

TEST(TestUtilities, test_context_release_interrupt_guard_condition) {
  auto context1 = std::make_shared<rclcpp::contexts::DefaultContext>();
  context1->init(0, nullptr);
  RCLCPP_SCOPE_EXIT(rclcpp::shutdown(context1););

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_ret_t ret = rcl_wait_set_init(
    &wait_set, 0, 2, 0, 0, 0, 0, context1->get_rcl_context().get(),
    rcl_get_default_allocator());
  ASSERT_EQ(RCL_RET_OK, ret);

  // Expected usage
  rcl_guard_condition_t * interrupt_guard_condition =
    context1->get_interrupt_guard_condition(&wait_set);
  EXPECT_NE(nullptr, interrupt_guard_condition);
  EXPECT_NO_THROW(context1->release_interrupt_guard_condition(&wait_set));

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_guard_condition_init, RCL_RET_ERROR);
    EXPECT_THROW(
      {interrupt_guard_condition = context1->get_interrupt_guard_condition(&wait_set);},
      rclcpp::exceptions::RCLError);
  }

  {
    interrupt_guard_condition = context1->get_interrupt_guard_condition(&wait_set);
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp", rcl_guard_condition_fini, RCL_RET_ERROR);
    EXPECT_THROW(
      {context1->release_interrupt_guard_condition(&wait_set);},
      rclcpp::exceptions::RCLError);
  }

  {
    interrupt_guard_condition = context1->get_interrupt_guard_condition(&wait_set);
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp", rcl_guard_condition_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW({context1->release_interrupt_guard_condition(&wait_set, std::nothrow);});
  }

  {
    EXPECT_THROW(
      context1->release_interrupt_guard_condition(nullptr),
      std::runtime_error);
  }

  // Test it works after restore mocks
  interrupt_guard_condition = context1->get_interrupt_guard_condition(&wait_set);
  EXPECT_NE(nullptr, interrupt_guard_condition);
  EXPECT_NO_THROW(context1->release_interrupt_guard_condition(&wait_set));
}

TEST(TestUtilities, test_context_init_shutdown_fails) {
  {
    auto context = std::make_shared<rclcpp::contexts::DefaultContext>();
    auto context_fail_init = std::make_shared<rclcpp::contexts::DefaultContext>();
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_init, RCL_RET_ERROR);
    EXPECT_THROW(context_fail_init->init(0, nullptr), rclcpp::exceptions::RCLError);
  }

  {
    auto context_fail_init = std::make_shared<rclcpp::contexts::DefaultContext>();
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_logging_configure_with_output_handler, RCL_RET_ERROR);
    EXPECT_THROW(context_fail_init->init(0, nullptr), rclcpp::exceptions::RCLError);
  }

  {
    auto context = std::make_shared<rclcpp::contexts::DefaultContext>();
    context->init(0, nullptr);
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
    // This will log a message, no throw expected
    EXPECT_NO_THROW(context->shutdown(""));
  }

  {
    auto context = std::make_shared<rclcpp::contexts::DefaultContext>();
    context->init(0, nullptr);
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp", rcl_shutdown, RCL_RET_ERROR);
    EXPECT_THROW(context->shutdown(""), rclcpp::exceptions::RCLError);
  }

  {
    auto context = std::make_shared<rclcpp::contexts::DefaultContext>();
    context->init(0, nullptr);
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp", rcl_logging_fini, RCL_RET_ERROR);
    // This will log a message, no throw expected
    EXPECT_NO_THROW(context->shutdown(""));
  }

  {
    auto context_to_destroy = std::make_shared<rclcpp::contexts::DefaultContext>();
    auto mock = mocking_utils::inject_on_return(
      "lib:rclcpp", rcl_trigger_guard_condition, RCL_RET_ERROR);
    // This will log a message, no throw expected
    EXPECT_NO_THROW({context_to_destroy.reset();});
  }
}

// Required for mocking_utils below
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, ==)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, !=)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, <)
MOCKING_UTILS_BOOL_OPERATOR_RETURNS_FALSE(rcutils_allocator_t, >)

TEST(TestUtilities, remove_ros_arguments_rcl_parse_arguments_failed) {
  const char * const argv[] = {
    "process_name",
    "-d", "--ros-args",
    "-r", "__ns:=/foo/bar",
    "-r", "__ns:=/fiz/buz",
    "--", "--foo=bar", "--baz"
  };
  int argc = sizeof(argv) / sizeof(const char *);

  auto mock = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_parse_arguments, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::remove_ros_arguments(argc, argv),
    rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, rcl_get_error_state(), "failed to parse arguments"));
}

TEST(TestUtilities, remove_ros_arguments_rcl_remove_ros_arguments_failed) {
  const char * const argv[] = {
    "process_name",
    "-d", "--ros-args",
    "-r", "__ns:=/foo/bar",
    "-r", "__ns:=/fiz/buz",
    "--", "--foo=bar", "--baz"
  };
  int argc = sizeof(argv) / sizeof(const char *);

  auto mock = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_remove_ros_arguments, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::remove_ros_arguments(argc, argv),
    rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, rcl_get_error_state(), ""));
}

TEST(TestUtilities, remove_ros_arguments_rcl_remove_ros_arguments_failed_and_fini) {
  const char * const argv[] = {
    "process_name",
    "-d", "--ros-args",
    "-r", "__ns:=/foo/bar",
    "-r", "__ns:=/fiz/buz",
    "--", "--foo=bar", "--baz"
  };
  int argc = sizeof(argv) / sizeof(const char *);

  auto mock = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_remove_ros_arguments, RCL_RET_ERROR);
  auto mock2 = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_arguments_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::remove_ros_arguments(argc, argv),
    rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, rcl_get_error_state(),
      ", failed also to cleanup parsed arguments, leaking memory: "));
}

TEST(TestUtilities, remove_ros_arguments_rcl_arguments_fini_failed) {
  const char * const argv[] = {
    "process_name",
    "-d", "--ros-args",
    "-r", "__ns:=/foo/bar",
    "-r", "__ns:=/fiz/buz",
    "--", "--foo=bar", "--baz"
  };
  int argc = sizeof(argv) / sizeof(const char *);

  auto mock = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_arguments_fini, RCL_RET_ERROR);
  RCLCPP_EXPECT_THROW_EQ(
    rclcpp::remove_ros_arguments(argc, argv),
    rclcpp::exceptions::RCLError(
      RCL_RET_ERROR, rcl_get_error_state(), "failed to cleanup parsed arguments, leaking memory"));
}
