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

#include <gtest/gtest.h>
#include <rcl/allocator.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcutils/strdup.h>

#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/parameter_map.hpp"

rcl_params_t *
make_params(std::vector<std::string> node_names)
{
  rcl_allocator_t alloc = rcl_get_default_allocator();
  rcl_params_t * c_params = rcl_yaml_node_struct_init(alloc);
  c_params->num_nodes = node_names.size();
  c_params->allocator = alloc;
  if (c_params->num_nodes) {
    // Copy node names
    for (size_t n = 0; n < node_names.size(); ++n) {
      c_params->node_names[n] = static_cast<char *>(alloc.allocate(
          sizeof(char) * (node_names[n].size() + 1), alloc.state));
      std::snprintf(c_params->node_names[n], node_names[n].size() + 1, "%s", node_names[n].c_str());
    }
    // zero init node params
    for (size_t n = 0; n < node_names.size(); ++n) {
      c_params->params[n].parameter_names = NULL;
      c_params->params[n].parameter_values = NULL;
      c_params->params[n].num_params = 0;

      c_params->descriptors[n].parameter_names = NULL;
      c_params->descriptors[n].parameter_descriptors = NULL;
      c_params->descriptors[n].num_params = 0;
    }
  }
  return c_params;
}

void
make_node_params(rcl_params_t * c_params, size_t node_idx, std::vector<std::string> param_names)
{
  rcl_allocator_t alloc = c_params->allocator;
  ASSERT_LT(node_idx, c_params->num_nodes);
  ASSERT_GT(param_names.size(), 0u);

  rcl_node_params_s * c_node_params = &(c_params->params[node_idx]);
  c_node_params->num_params = param_names.size();

  // Copy parameter names
  c_node_params->parameter_names = static_cast<char **>(
    alloc.allocate(sizeof(char *) * param_names.size(), alloc.state));
  for (size_t p = 0; p < param_names.size(); ++p) {
    const std::string & param_name = param_names[p];
    c_node_params->parameter_names[p] = static_cast<char *>(alloc.allocate(
        sizeof(char) * (param_name.size() + 1), alloc.state));
    std::snprintf(
      c_node_params->parameter_names[p], param_name.size() + 1, "%s", param_name.c_str());
  }
  // zero init parameter value
  c_node_params->parameter_values = static_cast<rcl_variant_t *>(alloc.allocate(
      sizeof(rcl_variant_t) * param_names.size(), alloc.state));
  for (size_t p = 0; p < param_names.size(); ++p) {
    c_node_params->parameter_values[p].bool_value = NULL;
    c_node_params->parameter_values[p].integer_value = NULL;
    c_node_params->parameter_values[p].double_value = NULL;
    c_node_params->parameter_values[p].string_value = NULL;
    c_node_params->parameter_values[p].byte_array_value = NULL;
    c_node_params->parameter_values[p].bool_array_value = NULL;
    c_node_params->parameter_values[p].integer_array_value = NULL;
    c_node_params->parameter_values[p].double_array_value = NULL;
    c_node_params->parameter_values[p].string_array_value = NULL;
  }
}

void
make_node_param_descriptors(
  rcl_params_t * c_params, size_t node_idx,
  std::vector<std::string> param_names)
{
  rcl_allocator_t alloc = c_params->allocator;
  ASSERT_LT(node_idx, c_params->num_nodes);
  ASSERT_GT(param_names.size(), 0u);

  rcl_node_params_descriptors_s * c_node_param_descriptors = &(c_params->descriptors[node_idx]);
  c_node_param_descriptors->num_params = param_names.size();

  // Copy parameter names
  c_node_param_descriptors->parameter_names = static_cast<char **>(
    alloc.allocate(sizeof(char *) * param_names.size(), alloc.state));
  for (size_t p = 0; p < param_names.size(); ++p) {
    const std::string & param_name = param_names[p];
    c_node_param_descriptors->parameter_names[p] = static_cast<char *>(alloc.allocate(
        sizeof(char) * (param_name.size() + 1), alloc.state));
    std::snprintf(
      c_node_param_descriptors->parameter_names[p], param_name.size() + 1, "%s",
      param_name.c_str());
  }
  // zero init parameter value
  c_node_param_descriptors->parameter_descriptors =
    static_cast<rcl_param_descriptor_t *>(alloc.allocate(
      sizeof(rcl_param_descriptor_t) * param_names.size(), alloc.state));
  for (size_t p = 0; p < param_names.size(); ++p) {
    c_node_param_descriptors->parameter_descriptors[p].name = NULL;
    c_node_param_descriptors->parameter_descriptors[p].read_only = NULL;
    c_node_param_descriptors->parameter_descriptors[p].type = NULL;
    c_node_param_descriptors->parameter_descriptors[p].description = NULL;
    c_node_param_descriptors->parameter_descriptors[p].additional_constraints = NULL;
    c_node_param_descriptors->parameter_descriptors[p].min_value_double = NULL;
    c_node_param_descriptors->parameter_descriptors[p].max_value_double = NULL;
    c_node_param_descriptors->parameter_descriptors[p].step_double = NULL;
    c_node_param_descriptors->parameter_descriptors[p].min_value_int = NULL;
    c_node_param_descriptors->parameter_descriptors[p].max_value_int = NULL;
    c_node_param_descriptors->parameter_descriptors[p].step_int = NULL;
  }
}

TEST(Test_parameter_map_from, null_c_parameter)
{
  EXPECT_THROW(rclcpp::parameter_map_from(NULL), rclcpp::exceptions::InvalidParametersException);
}

TEST(Test_parameter_map_from, null_node_names)
{
  rcl_params_t * c_params = make_params({});
  c_params->num_nodes = 1;

  EXPECT_THROW(
    rclcpp::parameter_map_from(c_params), rclcpp::exceptions::InvalidParametersException);

  c_params->num_nodes = 0;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, null_node_params)
{
  rcl_params_t * c_params = make_params({"foo"});
  std::snprintf(c_params->node_names[0], 3 + 1, "foo");
  auto allocated_params = c_params->params;
  c_params->params = NULL;

  EXPECT_THROW(
    rclcpp::parameter_map_from(c_params), rclcpp::exceptions::InvalidParametersException);

  c_params->params = allocated_params;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, null_node_name_in_node_names)
{
  rcl_params_t * c_params = make_params({"foo"});
  auto allocated_name = c_params->node_names[0];
  c_params->node_names[0] = NULL;

  EXPECT_THROW(
    rclcpp::parameter_map_from(c_params), rclcpp::exceptions::InvalidParametersException);

  c_params->node_names[0] = allocated_name;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, null_node_param_value)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_params(c_params, 0, {"bar"});
  make_node_param_descriptors(c_params, 0, {"baz"});

  EXPECT_THROW(
    rclcpp::parameter_map_from(c_params), rclcpp::exceptions::InvalidParameterValueException);

  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, null_node_param_name)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_params(c_params, 0, {"bar"});
  auto allocated_name = c_params->params[0].parameter_names[0];
  c_params->params[0].parameter_names[0] = NULL;

  make_node_param_descriptors(c_params, 0, {"baz"});
  auto allocated_descriptor_name = c_params->descriptors[0].parameter_names[0];
  c_params->descriptors[0].parameter_names[0] = NULL;

  EXPECT_THROW(
    rclcpp::parameter_map_from(c_params), rclcpp::exceptions::InvalidParametersException);

  c_params->params[0].parameter_names[0] = allocated_name;
  c_params->descriptors[0].parameter_names[0] = allocated_descriptor_name;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, bool_param_value)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_params(c_params, 0, {"true_bool", "false_bool"});
  bool true_bool = true;
  bool false_bool = false;
  c_params->params[0].parameter_values[0].bool_value = &true_bool;
  c_params->params[0].parameter_values[1].bool_value = &false_bool;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo");
  EXPECT_TRUE(params.count("true_bool"));
  EXPECT_TRUE(params.at("true_bool").value.get<bool>());
  EXPECT_TRUE(params.count("false_bool"));
  EXPECT_FALSE(params.at("false_bool").value.get<bool>());

  c_params->params[0].parameter_values[0].bool_value = NULL;
  c_params->params[0].parameter_values[1].bool_value = NULL;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, integer_param_value)
{
  rcl_params_t * c_params = make_params({"bar"});
  make_node_params(c_params, 0, {"positive.int", "negative.int"});
  int64_t positive_int = 42;
  int64_t negative_int = -12345;
  c_params->params[0].parameter_values[0].integer_value = &positive_int;
  c_params->params[0].parameter_values[1].integer_value = &negative_int;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/bar");
  EXPECT_TRUE(params.count("positive.int"));
  EXPECT_EQ(42, params.at("positive.int").value.get<int64_t>());
  EXPECT_TRUE(params.count("negative.int"));
  EXPECT_EQ(-12345, params.at("negative.int").value.get<int64_t>());

  c_params->params[0].parameter_values[0].integer_value = NULL;
  c_params->params[0].parameter_values[1].integer_value = NULL;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, double_param_value)
{
  rcl_params_t * c_params = make_params({"foo/bar"});
  make_node_params(c_params, 0, {"positive.double", "negative.double"});
  double positive_double = 3.14;
  double negative_double = -2.718;
  c_params->params[0].parameter_values[0].double_value = &positive_double;
  c_params->params[0].parameter_values[1].double_value = &negative_double;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo/bar");
  EXPECT_TRUE(params.count("positive.double"));
  EXPECT_DOUBLE_EQ(3.14, params.at("positive.double").value.get<double>());
  EXPECT_TRUE(params.count("negative.double"));
  EXPECT_DOUBLE_EQ(-2.718, params.at("negative.double").value.get<double>());

  c_params->params[0].parameter_values[0].double_value = NULL;
  c_params->params[0].parameter_values[1].double_value = NULL;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, string_param_value)
{
  rcl_params_t * c_params = make_params({"/foo/bar"});
  make_node_params(c_params, 0, {"string_param"});
  std::string hello_world = "hello world";
  char * c_hello_world = new char[hello_world.length() + 1];
  std::snprintf(c_hello_world, hello_world.size() + 1, "%s", hello_world.c_str());
  c_params->params[0].parameter_values[0].string_value = c_hello_world;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo/bar");
  EXPECT_TRUE(params.count("string_param"));
  EXPECT_STREQ(hello_world.c_str(),
    params.at("string_param").value.get<std::string>().c_str());

  c_params->params[0].parameter_values[0].string_value = NULL;
  delete[] c_hello_world;
  rcl_yaml_node_struct_fini(c_params);
}

#define MAKE_ARRAY_VALUE(VAR, TYPE, V1, V2) \
  do { \
    VAR.values = new TYPE[2]; \
    VAR.size = 2; \
    VAR.values[0] = V1; \
    VAR.values[1] = V2; \
  } while (false)

#define FREE_ARRAY_VALUE(VAR) \
  do { \
    delete[] VAR.values; \
  } while (false)

TEST(Test_parameter_map_from, byte_array_param_value)
{
  rcl_params_t * c_params = make_params({"/foobar"});
  make_node_params(c_params, 0, {"byte_array_param"});
  rcl_byte_array_t c_byte_array;
  MAKE_ARRAY_VALUE(c_byte_array, uint8_t, 0xf0, 0xaa);
  c_params->params[0].parameter_values[0].byte_array_value = &c_byte_array;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foobar");
  EXPECT_TRUE(params.count("byte_array_param"));
  std::vector<uint8_t> byte_array =
    params.at("byte_array_param").value.get<std::vector<uint8_t>>();
  ASSERT_EQ(2u, byte_array.size());
  EXPECT_EQ(0xf0, byte_array.at(0));
  EXPECT_EQ(0xaa, byte_array.at(1));

  c_params->params[0].parameter_values[0].byte_array_value = NULL;
  FREE_ARRAY_VALUE(c_byte_array);
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, bool_array_param_value)
{
  rcl_params_t * c_params = make_params({"foo/bar/baz"});
  make_node_params(c_params, 0, {"bool_array_param"});
  rcl_bool_array_t c_bool_array;
  MAKE_ARRAY_VALUE(c_bool_array, bool, true, false);
  c_params->params[0].parameter_values[0].bool_array_value = &c_bool_array;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo/bar/baz");
  EXPECT_TRUE(params.count("bool_array_param"));
  std::vector<bool> bool_array = params.at("bool_array_param").value.get<std::vector<bool>>();
  ASSERT_EQ(2u, bool_array.size());
  EXPECT_TRUE(bool_array.at(0));
  EXPECT_FALSE(bool_array.at(1));

  c_params->params[0].parameter_values[0].bool_array_value = NULL;
  FREE_ARRAY_VALUE(c_bool_array);
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, integer_array_param_value)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_params(c_params, 0, {"integer_array_param"});
  rcl_int64_array_t c_integer_array;
  MAKE_ARRAY_VALUE(c_integer_array, int64_t, 42, -12345);
  c_params->params[0].parameter_values[0].integer_array_value = &c_integer_array;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo");
  EXPECT_TRUE(params.count("integer_array_param"));
  std::vector<int64_t> integer_array =
    params.at("integer_array_param").value.get<std::vector<int64_t>>();
  ASSERT_EQ(2u, integer_array.size());
  EXPECT_EQ(42, integer_array.at(0));
  EXPECT_EQ(-12345, integer_array.at(1));

  c_params->params[0].parameter_values[0].integer_array_value = NULL;
  FREE_ARRAY_VALUE(c_integer_array);
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, double_array_param_value)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_params(c_params, 0, {"double_array_param"});
  rcl_double_array_t c_double_array;
  MAKE_ARRAY_VALUE(c_double_array, double, 3.14, -2.718);
  c_params->params[0].parameter_values[0].double_array_value = &c_double_array;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo");
  EXPECT_TRUE(params.count("double_array_param"));
  std::vector<double> double_array =
    params.at("double_array_param").value.get<std::vector<double>>();
  ASSERT_EQ(2u, double_array.size());
  EXPECT_DOUBLE_EQ(3.14, double_array.at(0));
  EXPECT_DOUBLE_EQ(-2.718, double_array.at(1));

  c_params->params[0].parameter_values[0].double_array_value = NULL;
  FREE_ARRAY_VALUE(c_double_array);
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, string_array_param_value)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_params(c_params, 0, {"string_array_param"});
  rcutils_string_array_t c_string_array = rcutils_get_zero_initialized_string_array();
  ASSERT_EQ(RCUTILS_RET_OK, rcutils_string_array_init(&c_string_array, 2, &(c_params->allocator)));
  c_string_array.data[0] = rcutils_strdup("Hello", c_params->allocator);
  c_string_array.data[1] = rcutils_strdup("World", c_params->allocator);
  c_params->params[0].parameter_values[0].string_array_value = &c_string_array;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo");
  EXPECT_TRUE(params.count("string_array_param"));
  std::vector<std::string> string_array =
    params.at("string_array_param").value.get<std::vector<std::string>>();
  ASSERT_EQ(2u, string_array.size());
  EXPECT_STREQ("Hello", string_array.at(0).c_str());
  EXPECT_STREQ("World", string_array.at(1).c_str());

  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&c_string_array));
  c_params->params[0].parameter_values[0].string_array_value = NULL;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, descriptor_integer_range)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_param_descriptors(c_params, 0, {"bar", "baz", "foobar"});
  int64_t min_value = -1234;
  int64_t max_value = 99;
  int64_t step = 2;
  c_params->descriptors[0].parameter_descriptors[0].name = const_cast<char *>("bar");
  c_params->descriptors[0].parameter_descriptors[0].min_value_int = &min_value;
  c_params->descriptors[0].parameter_descriptors[0].max_value_int = &max_value;
  c_params->descriptors[0].parameter_descriptors[0].step_int = &step;
  c_params->descriptors[0].parameter_descriptors[0].description =
    const_cast<char *>("Integer Range Descriptor");
  c_params->descriptors[0].parameter_descriptors[0].additional_constraints =
    const_cast<char *>("Even numbers only");
  c_params->descriptors[0].parameter_descriptors[1].name = const_cast<char *>("baz");
  c_params->descriptors[0].parameter_descriptors[1].min_value_int = &min_value;
  c_params->descriptors[0].parameter_descriptors[2].name = const_cast<char *>("foobar");
  c_params->descriptors[0].parameter_descriptors[2].max_value_int = &max_value;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo");
  EXPECT_TRUE(params.count("bar"));
  EXPECT_STREQ("Integer Range Descriptor", params.at("bar").descriptor.description.c_str());
  EXPECT_STREQ("Even numbers only", params.at("bar").descriptor.additional_constraints.c_str());
  EXPECT_EQ(-1234, params.at("bar").descriptor.integer_range[0].from_value);
  EXPECT_EQ(99, params.at("bar").descriptor.integer_range[0].to_value);
  EXPECT_EQ(2U, params.at("bar").descriptor.integer_range[0].step);
  EXPECT_TRUE(params.count("baz"));
  EXPECT_EQ(-1234, params.at("baz").descriptor.integer_range[0].from_value);
  EXPECT_EQ(std::numeric_limits<int64_t>::max(), params.at("baz").descriptor.integer_range[0].to_value);
  EXPECT_TRUE(params.count("foobar"));
  EXPECT_EQ(std::numeric_limits<int64_t>::min(), params.at(
      "foobar").descriptor.integer_range[0].from_value);
  EXPECT_EQ(99, params.at("foobar").descriptor.integer_range[0].to_value);

  c_params->descriptors[0].parameter_descriptors[0].name = NULL;
  c_params->descriptors[0].parameter_descriptors[0].min_value_int = NULL;
  c_params->descriptors[0].parameter_descriptors[0].max_value_int = NULL;
  c_params->descriptors[0].parameter_descriptors[0].step_int = NULL;
  c_params->descriptors[0].parameter_descriptors[0].description = NULL;
  c_params->descriptors[0].parameter_descriptors[0].additional_constraints = NULL;
  c_params->descriptors[0].parameter_descriptors[1].name = NULL;
  c_params->descriptors[0].parameter_descriptors[1].min_value_int = NULL;
  c_params->descriptors[0].parameter_descriptors[2].name = NULL;
  c_params->descriptors[0].parameter_descriptors[2].max_value_int = NULL;

  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, descriptor_double_range)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_param_descriptors(c_params, 0, {"bar", "baz", "foobar"});
  double min_value = -1000.0;
  double max_value = 500.0;
  double step = 5.0;
  c_params->descriptors[0].parameter_descriptors[0].name = const_cast<char *>("bar");
  c_params->descriptors[0].parameter_descriptors[0].min_value_double = &min_value;
  c_params->descriptors[0].parameter_descriptors[0].max_value_double = &max_value;
  c_params->descriptors[0].parameter_descriptors[0].step_double = &step;
  c_params->descriptors[0].parameter_descriptors[0].description =
    const_cast<char *>("Double Range Descriptor");
  c_params->descriptors[0].parameter_descriptors[0].additional_constraints =
    const_cast<char *>("Multiples of 5");
  c_params->descriptors[0].parameter_descriptors[1].name = const_cast<char *>("baz");
  c_params->descriptors[0].parameter_descriptors[1].min_value_double = &min_value;
  c_params->descriptors[0].parameter_descriptors[2].name = const_cast<char *>("foobar");
  c_params->descriptors[0].parameter_descriptors[2].max_value_double = &max_value;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo");
  EXPECT_TRUE(params.count("bar"));
  EXPECT_STREQ("Double Range Descriptor", params.at("bar").descriptor.description.c_str());
  EXPECT_STREQ("Multiples of 5", params.at("bar").descriptor.additional_constraints.c_str());
  EXPECT_DOUBLE_EQ(-1000.0, params.at("bar").descriptor.floating_point_range[0].from_value);
  EXPECT_DOUBLE_EQ(500.0, params.at("bar").descriptor.floating_point_range[0].to_value);
  EXPECT_DOUBLE_EQ(5.0, params.at("bar").descriptor.floating_point_range[0].step);
  EXPECT_TRUE(params.count("baz"));
  EXPECT_DOUBLE_EQ(-1000.0, params.at("baz").descriptor.floating_point_range[0].from_value);
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::max(),
    params.at("baz").descriptor.floating_point_range[0].to_value);
  EXPECT_TRUE(params.count("foobar"));
  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::lowest(), params.at(
      "foobar").descriptor.floating_point_range[0].from_value);
  EXPECT_DOUBLE_EQ(500.0, params.at("foobar").descriptor.floating_point_range[0].to_value);

  c_params->descriptors[0].parameter_descriptors[0].name = NULL;
  c_params->descriptors[0].parameter_descriptors[0].min_value_double = NULL;
  c_params->descriptors[0].parameter_descriptors[0].max_value_double = NULL;
  c_params->descriptors[0].parameter_descriptors[0].step_double = NULL;
  c_params->descriptors[0].parameter_descriptors[0].description = NULL;
  c_params->descriptors[0].parameter_descriptors[0].additional_constraints = NULL;
  c_params->descriptors[0].parameter_descriptors[1].name = NULL;
  c_params->descriptors[0].parameter_descriptors[1].min_value_double = NULL;
  c_params->descriptors[0].parameter_descriptors[2].name = NULL;
  c_params->descriptors[0].parameter_descriptors[2].max_value_double = NULL;

  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, descriptor_mixed_range_types)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_param_descriptors(c_params, 0, {"bar"});
  int64_t min_value = 5;
  double max_value = 25.0;

  c_params->descriptors[0].parameter_descriptors[0].name = const_cast<char *>("bar");
  c_params->descriptors[0].parameter_descriptors[0].min_value_int = &min_value;
  c_params->descriptors[0].parameter_descriptors[0].max_value_double = &max_value;

  EXPECT_THROW(
    rclcpp::parameter_map_from(c_params), rclcpp::exceptions::InvalidParameterValueException);

  c_params->descriptors[0].parameter_descriptors[0].name = NULL;
  c_params->descriptors[0].parameter_descriptors[0].min_value_int = NULL;
  c_params->descriptors[0].parameter_descriptors[0].max_value_double = NULL;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, descriptor_read_only)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_param_descriptors(c_params, 0, {"bar", "baz"});
  bool read_only_true = true;
  c_params->descriptors[0].parameter_descriptors[0].name = const_cast<char *>("bar");
  c_params->descriptors[0].parameter_descriptors[0].read_only = &read_only_true;
  c_params->descriptors[0].parameter_descriptors[0].description =
    const_cast<char *>("read-only param");
  c_params->descriptors[0].parameter_descriptors[1].name = const_cast<char *>("baz");
  c_params->descriptors[0].parameter_descriptors[1].description =
    const_cast<char *>("not read-only");

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params);
  const rclcpp::ParameterAndDescriptor & params = map.at("/foo");
  EXPECT_TRUE(params.count("bar"));
  EXPECT_STREQ("read-only param", params.at("bar").descriptor.description.c_str());
  EXPECT_TRUE(params.at("bar").descriptor.read_only);
  EXPECT_TRUE(params.count("baz"));
  EXPECT_STREQ("not read-only", params.at("baz").descriptor.description.c_str());
  EXPECT_FALSE(params.at("baz").descriptor.read_only);


  c_params->descriptors[0].parameter_descriptors[0].name = NULL;
  c_params->descriptors[0].parameter_descriptors[0].read_only = NULL;
  c_params->descriptors[0].parameter_descriptors[0].description = NULL;
  c_params->descriptors[0].parameter_descriptors[1].name = NULL;
  c_params->descriptors[0].parameter_descriptors[1].description = NULL;
  rcl_yaml_node_struct_fini(c_params);
}
