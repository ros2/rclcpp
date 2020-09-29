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
#include <string>
#include <vector>

#include "rcl/allocator.h"
#include "rcl/arguments.h"
#include "rcl/remap.h"

#include "rclcpp/node_options.hpp"

#include "../mocking_utils/patch.hpp"


TEST(TestNodeOptions, ros_args_only) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::NodeOptions(allocator)
    .arguments({"--ros-args", "-r", "__node:=some_node", "-r", "__ns:=/some_ns"});

  const rcl_node_options_t * rcl_options = options.get_rcl_node_options();
  ASSERT_TRUE(rcl_options != nullptr);
  ASSERT_EQ(0, rcl_arguments_get_count_unparsed(&rcl_options->arguments));
  ASSERT_EQ(0, rcl_arguments_get_count_unparsed_ros(&rcl_options->arguments));

  char * node_name = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_remap_node_name(
      &rcl_options->arguments, nullptr, "my_node", allocator, &node_name));
  ASSERT_TRUE(node_name != nullptr);
  EXPECT_STREQ("some_node", node_name);
  allocator.deallocate(node_name, allocator.state);

  char * namespace_name = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_remap_node_namespace(
      &rcl_options->arguments, nullptr, "my_ns", allocator, &namespace_name));
  ASSERT_TRUE(namespace_name != nullptr);
  EXPECT_STREQ("/some_ns", namespace_name);
  allocator.deallocate(namespace_name, allocator.state);
}

TEST(TestNodeOptions, ros_args_and_non_ros_args) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::NodeOptions(allocator).arguments(
  {
    "--non-ros-flag", "--ros-args", "-r", "__node:=some_node",
    "-r", "__ns:=/some_ns", "--", "non-ros-arg"});

  const rcl_node_options_t * rcl_options = options.get_rcl_node_options();
  ASSERT_TRUE(rcl_options != nullptr);
  ASSERT_EQ(0, rcl_arguments_get_count_unparsed_ros(&rcl_options->arguments));
  ASSERT_EQ(2, rcl_arguments_get_count_unparsed(&rcl_options->arguments));

  char * node_name = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_remap_node_name(
      &rcl_options->arguments, nullptr, "my_node", allocator, &node_name));
  ASSERT_TRUE(node_name != nullptr);
  EXPECT_STREQ("some_node", node_name);
  allocator.deallocate(node_name, allocator.state);

  char * namespace_name = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_remap_node_namespace(
      &rcl_options->arguments, nullptr, "my_ns", allocator, &namespace_name));
  ASSERT_TRUE(namespace_name != nullptr);
  EXPECT_STREQ("/some_ns", namespace_name);
  allocator.deallocate(namespace_name, allocator.state);

  int * output_indices = nullptr;
  EXPECT_EQ(
    RCL_RET_OK, rcl_arguments_get_unparsed(
      &rcl_options->arguments, allocator, &output_indices));
  ASSERT_TRUE(output_indices != nullptr);
  const std::vector<std::string> & args = options.arguments();
  EXPECT_EQ("--non-ros-flag", args[output_indices[0]]);
  EXPECT_EQ("non-ros-arg", args[output_indices[1]]);
  allocator.deallocate(output_indices, allocator.state);
}

TEST(TestNodeOptions, bad_ros_args) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto options = rclcpp::NodeOptions(allocator)
    .arguments({"--ros-args", "-r", "foo:="});

  EXPECT_THROW(
    options.get_rcl_node_options(),
    rclcpp::exceptions::RCLInvalidROSArgsError);

  options.arguments({"--ros-args", "-r", "foo:=bar", "not-a-ros-arg"});
  EXPECT_THROW(
    options.get_rcl_node_options(),
    rclcpp::exceptions::UnknownROSArgsError);
}

TEST(TestNodeOptions, use_global_arguments) {
  {
    auto options = rclcpp::NodeOptions();
    EXPECT_TRUE(options.use_global_arguments());
    EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);
  }

  {
    auto options = rclcpp::NodeOptions().use_global_arguments(false);
    EXPECT_FALSE(options.use_global_arguments());
    EXPECT_FALSE(options.get_rcl_node_options()->use_global_arguments);
  }

  {
    auto options = rclcpp::NodeOptions().use_global_arguments(true);
    EXPECT_TRUE(options.use_global_arguments());
    EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);
  }

  {
    auto options = rclcpp::NodeOptions();
    EXPECT_TRUE(options.use_global_arguments());
    EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);
    options.use_global_arguments(false);
    EXPECT_FALSE(options.use_global_arguments());
    EXPECT_FALSE(options.get_rcl_node_options()->use_global_arguments);
    options.use_global_arguments(true);
    EXPECT_TRUE(options.use_global_arguments());
    EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);
  }
}

TEST(TestNodeOptions, enable_rosout) {
  {
    auto options = rclcpp::NodeOptions();
    EXPECT_TRUE(options.enable_rosout());
    EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);
  }

  {
    auto options = rclcpp::NodeOptions().enable_rosout(false);
    EXPECT_FALSE(options.enable_rosout());
    EXPECT_FALSE(options.get_rcl_node_options()->enable_rosout);
  }

  {
    auto options = rclcpp::NodeOptions().enable_rosout(true);
    EXPECT_TRUE(options.enable_rosout());
    EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);
  }

  {
    auto options = rclcpp::NodeOptions();
    EXPECT_TRUE(options.enable_rosout());
    EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);
    options.enable_rosout(false);
    EXPECT_FALSE(options.enable_rosout());
    EXPECT_FALSE(options.get_rcl_node_options()->enable_rosout);
    options.enable_rosout(true);
    EXPECT_TRUE(options.enable_rosout());
    EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);
  }
}

TEST(TestNodeOptions, copy) {
  std::vector<std::string> expected_args{"--unknown-flag", "arg"};
  auto options = rclcpp::NodeOptions().arguments(expected_args).use_global_arguments(false);
  const rcl_node_options_t * rcl_options = options.get_rcl_node_options();

  {
    rclcpp::NodeOptions copied_options = options;
    EXPECT_FALSE(copied_options.use_global_arguments());
    EXPECT_EQ(expected_args, copied_options.arguments());
    const rcl_node_options_t * copied_rcl_options = copied_options.get_rcl_node_options();
    EXPECT_EQ(copied_rcl_options->use_global_arguments, rcl_options->use_global_arguments);
    EXPECT_EQ(
      rcl_arguments_get_count_unparsed(&copied_rcl_options->arguments),
      rcl_arguments_get_count_unparsed(&rcl_options->arguments));
  }

  {
    auto other_options = rclcpp::NodeOptions().use_global_arguments(true);
    (void)other_options.get_rcl_node_options();  // force C structure initialization
    other_options = options;
    EXPECT_FALSE(other_options.use_global_arguments());
    EXPECT_EQ(expected_args, other_options.arguments());
    const rcl_node_options_t * other_rcl_options = other_options.get_rcl_node_options();
    EXPECT_EQ(other_rcl_options->use_global_arguments, rcl_options->use_global_arguments);
    EXPECT_EQ(
      rcl_arguments_get_count_unparsed(&other_rcl_options->arguments),
      rcl_arguments_get_count_unparsed(&rcl_options->arguments));
  }
}

TEST(TestNodeOptions, append_parameter_override) {
  std::vector<std::string> expected_args{"--unknown-flag", "arg"};
  auto options = rclcpp::NodeOptions().arguments(expected_args).use_global_arguments(false);
  rclcpp::Parameter parameter("some_parameter", 10);
  options.append_parameter_override("some_parameter", 10);
  EXPECT_EQ(1u, options.parameter_overrides().size());
  EXPECT_EQ(std::string("some_parameter"), options.parameter_overrides()[0].get_name());
}

TEST(TestNodeOptions, rcl_node_options_fini_error) {
  auto mock = mocking_utils::inject_on_return("lib:rclcpp", rcl_node_options_fini, RCL_RET_ERROR);
  auto options = std::make_shared<rclcpp::NodeOptions>();
  // Necessary to setup internal pointer
  ASSERT_NE(nullptr, options->get_rcl_node_options());
  // If fini fails, this should just log an error and continue
  EXPECT_NO_THROW(options.reset());
}

TEST(TestNodeOptions, bool_setters_and_getters) {
  rclcpp::NodeOptions options;

  options.use_global_arguments(false);
  EXPECT_FALSE(options.use_global_arguments());
  EXPECT_FALSE(options.get_rcl_node_options()->use_global_arguments);
  options.use_global_arguments(true);
  EXPECT_TRUE(options.use_global_arguments());
  EXPECT_TRUE(options.get_rcl_node_options()->use_global_arguments);

  options.enable_rosout(false);
  EXPECT_FALSE(options.enable_rosout());
  EXPECT_FALSE(options.get_rcl_node_options()->enable_rosout);
  options.enable_rosout(true);
  EXPECT_TRUE(options.enable_rosout());
  EXPECT_TRUE(options.get_rcl_node_options()->enable_rosout);

  options.use_intra_process_comms(false);
  EXPECT_FALSE(options.use_intra_process_comms());
  options.use_intra_process_comms(true);
  EXPECT_TRUE(options.use_intra_process_comms());

  options.enable_topic_statistics(false);
  EXPECT_FALSE(options.enable_topic_statistics());
  options.enable_topic_statistics(true);
  EXPECT_TRUE(options.enable_topic_statistics());

  options.start_parameter_services(false);
  EXPECT_FALSE(options.start_parameter_services());
  options.start_parameter_services(true);
  EXPECT_TRUE(options.start_parameter_services());

  options.allow_undeclared_parameters(false);
  EXPECT_FALSE(options.allow_undeclared_parameters());
  options.allow_undeclared_parameters(true);
  EXPECT_TRUE(options.allow_undeclared_parameters());

  options.start_parameter_event_publisher(false);
  EXPECT_FALSE(options.start_parameter_event_publisher());
  options.start_parameter_event_publisher(true);
  EXPECT_TRUE(options.start_parameter_event_publisher());

  options.automatically_declare_parameters_from_overrides(false);
  EXPECT_FALSE(options.automatically_declare_parameters_from_overrides());
  options.automatically_declare_parameters_from_overrides(true);
  EXPECT_TRUE(options.automatically_declare_parameters_from_overrides());
}

TEST(TestNodeOptions, parameter_event_qos) {
  rclcpp::NodeOptions options;
  rclcpp::QoS qos1(1);
  rclcpp::QoS qos2(2);
  EXPECT_NE(qos1, options.parameter_event_qos());
  EXPECT_NE(qos2, options.parameter_event_qos());
  options.parameter_event_qos(qos1);
  EXPECT_EQ(qos1, options.parameter_event_qos());
  options.parameter_event_qos(qos2);
  EXPECT_EQ(qos2, options.parameter_event_qos());
}

TEST(TestNodeOptions, parameter_event_publisher_options) {
  rclcpp::NodeOptions options;
  rclcpp::PublisherOptionsBase publisher_options;
  publisher_options.use_default_callbacks = true;
  options.parameter_event_publisher_options(publisher_options);
  EXPECT_TRUE(options.parameter_event_publisher_options().use_default_callbacks);

  publisher_options.use_default_callbacks = false;
  options.parameter_event_publisher_options(publisher_options);
  EXPECT_FALSE(options.parameter_event_publisher_options().use_default_callbacks);
}

TEST(TestNodeOptions, set_get_allocator) {
  rclcpp::NodeOptions options;
  EXPECT_NE(nullptr, options.allocator().allocate);
  EXPECT_NE(nullptr, options.allocator().deallocate);
  EXPECT_NE(nullptr, options.allocator().reallocate);
  EXPECT_NE(nullptr, options.allocator().zero_allocate);

  rcl_allocator_t fake_allocator;
  fake_allocator.allocate = [](size_t, void *) -> void * {return nullptr;};
  fake_allocator.deallocate = [](void *, void *) {};
  fake_allocator.reallocate = [](void *, size_t, void *) -> void * {return nullptr;};
  fake_allocator.zero_allocate = [](size_t, size_t, void *) -> void * {return nullptr;};
  fake_allocator.state = rcl_get_default_allocator().state;

  options.allocator(fake_allocator);
  EXPECT_EQ(fake_allocator.allocate, options.allocator().allocate);
  EXPECT_EQ(fake_allocator.deallocate, options.allocator().deallocate);
  EXPECT_EQ(fake_allocator.reallocate, options.allocator().reallocate);
  EXPECT_EQ(fake_allocator.zero_allocate, options.allocator().zero_allocate);
  EXPECT_EQ(fake_allocator.state, options.allocator().state);

  // Check invalid allocator
  EXPECT_THROW(options.get_rcl_node_options(), std::bad_alloc);
}
