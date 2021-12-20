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
#include <string>
#include <unordered_map>
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

  EXPECT_THROW(
    rclcpp::parameter_map_from(c_params), rclcpp::exceptions::InvalidParametersException);

  c_params->params[0].parameter_names[0] = allocated_name;
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
  const std::vector<rclcpp::Parameter> & params = map.at("/foo");
  EXPECT_STREQ("true_bool", params.at(0).get_name().c_str());
  EXPECT_TRUE(params.at(0).get_value<bool>());
  EXPECT_STREQ("false_bool", params.at(1).get_name().c_str());
  EXPECT_FALSE(params.at(1).get_value<bool>());

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
  const std::vector<rclcpp::Parameter> & params = map.at("/bar");
  EXPECT_STREQ("positive.int", params.at(0).get_name().c_str());
  EXPECT_EQ(42, params.at(0).get_value<int64_t>());
  EXPECT_STREQ("negative.int", params.at(1).get_name().c_str());
  EXPECT_EQ(-12345, params.at(1).get_value<int64_t>());

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
  const std::vector<rclcpp::Parameter> & params = map.at("/foo/bar");
  EXPECT_STREQ("positive.double", params.at(0).get_name().c_str());
  EXPECT_DOUBLE_EQ(3.14, params.at(0).get_value<double>());
  EXPECT_STREQ("negative.double", params.at(1).get_name().c_str());
  EXPECT_DOUBLE_EQ(-2.718, params.at(1).get_value<double>());

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
  const std::vector<rclcpp::Parameter> & params = map.at("/foo/bar");
  EXPECT_STREQ("string_param", params.at(0).get_name().c_str());
  EXPECT_STREQ(hello_world.c_str(), params.at(0).get_value<std::string>().c_str());

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
  const std::vector<rclcpp::Parameter> & params = map.at("/foobar");
  EXPECT_STREQ("byte_array_param", params.at(0).get_name().c_str());
  std::vector<uint8_t> byte_array = params.at(0).get_value<std::vector<uint8_t>>();
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
  const std::vector<rclcpp::Parameter> & params = map.at("/foo/bar/baz");
  EXPECT_STREQ("bool_array_param", params.at(0).get_name().c_str());
  std::vector<bool> bool_array = params.at(0).get_value<std::vector<bool>>();
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
  const std::vector<rclcpp::Parameter> & params = map.at("/foo");
  EXPECT_STREQ("integer_array_param", params.at(0).get_name().c_str());
  std::vector<int64_t> integer_array = params.at(0).get_value<std::vector<int64_t>>();
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
  const std::vector<rclcpp::Parameter> & params = map.at("/foo");
  EXPECT_STREQ("double_array_param", params.at(0).get_name().c_str());
  std::vector<double> double_array = params.at(0).get_value<std::vector<double>>();
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
  const std::vector<rclcpp::Parameter> & params = map.at("/foo");
  EXPECT_STREQ("string_array_param", params.at(0).get_name().c_str());
  std::vector<std::string> string_array = params.at(0).get_value<std::vector<std::string>>();
  ASSERT_EQ(2u, string_array.size());
  EXPECT_STREQ("Hello", string_array.at(0).c_str());
  EXPECT_STREQ("World", string_array.at(1).c_str());

  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&c_string_array));
  c_params->params[0].parameter_values[0].string_array_value = NULL;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, one_node_one_param_by_node_fqn)
{
  rcl_params_t * c_params = make_params({"foo"});
  make_node_params(c_params, 0, {"string_param"});

  std::string hello_world = "hello world";
  char * c_hello_world = new char[hello_world.length() + 1];
  std::snprintf(c_hello_world, hello_world.size() + 1, "%s", hello_world.c_str());
  c_params->params[0].parameter_values[0].string_value = c_hello_world;

  rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params, "/foo");
  const std::vector<rclcpp::Parameter> & params = map.at("/foo");
  EXPECT_STREQ("string_param", params.at(0).get_name().c_str());
  EXPECT_STREQ(hello_world.c_str(), params.at(0).get_value<std::string>().c_str());

  c_params->params[0].parameter_values[0].string_value = NULL;
  delete[] c_hello_world;
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, multi_nodes_same_param_name_by_node_fqn)
{
  std::vector<std::string> node_names_keys = {
    "/**",              // index: 0
    "/*",               // index: 1
    "/**/node",         // index: 2
    "/*/node",          // index: 3
    "/ns/node"          // index: 4
  };

  rcl_params_t * c_params = make_params(node_names_keys);

  std::vector<char *> param_values;
  for (size_t i = 0; i < node_names_keys.size(); ++i) {
    make_node_params(c_params, i, {"string_param"});
    std::string hello_world = "hello world" + std::to_string(i);
    char * c_hello_world = new char[hello_world.length() + 1];
    std::snprintf(c_hello_world, hello_world.size() + 1, "%s", hello_world.c_str());
    c_params->params[i].parameter_values[0].string_value = c_hello_world;
    param_values.push_back(c_hello_world);
  }

  std::unordered_map<std::string, std::vector<size_t>> node_fqn_expected = {
    {"/ns/foo/another_node", {0}},
    {"/another", {0, 1}},
    {"/node", {0, 1, 2}},
    {"/another_ns/node", {0, 2, 3}},
    {"/ns/node", {0, 2, 3, 4}},
  };

  for (auto & kv : node_fqn_expected) {
    rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params, kv.first.c_str());
    const std::vector<rclcpp::Parameter> & params = map.at(kv.first);

    EXPECT_EQ(kv.second.size(), params.size());
    for (size_t i = 0; i < params.size(); ++i) {
      std::string param_value = "hello world" + std::to_string(kv.second[i]);
      EXPECT_STREQ("string_param", params.at(i).get_name().c_str());
      EXPECT_STREQ(param_value.c_str(), params.at(i).get_value<std::string>().c_str());
    }
  }

  for (size_t i = 0; i < node_names_keys.size(); ++i) {
    c_params->params[i].parameter_values[0].string_value = NULL;
  }
  for (auto c_hello_world : param_values) {
    delete[] c_hello_world;
  }
  rcl_yaml_node_struct_fini(c_params);
}

TEST(Test_parameter_map_from, multi_nodes_diff_param_name_by_node_fqn)
{
  std::vector<std::string> node_names_keys = {
    "/**",              // index: 0
    "/*",               // index: 1
    "/**/node",         // index: 2
    "/*/node",          // index: 3
    "/ns/**",           // index: 4
    "/ns/*",            // index: 5
    "/ns/**/node",      // index: 6
    "/ns/*/node",       // index: 7
    "/ns/**/a/*/node",  // index: 8
    "/ns/node"          // index: 9
  };

  rcl_params_t * c_params = make_params(node_names_keys);

  for (size_t i = 0; i < node_names_keys.size(); ++i) {
    std::string param_name = "string_param" + std::to_string(i);
    make_node_params(c_params, i, {param_name});
  }

  std::string hello_world = "hello world";
  char * c_hello_world = new char[hello_world.length() + 1];
  std::snprintf(c_hello_world, hello_world.size() + 1, "%s", hello_world.c_str());

  for (size_t i = 0; i < node_names_keys.size(); ++i) {
    c_params->params[i].parameter_values[0].string_value = c_hello_world;
  }

  std::unordered_map<std::string, std::vector<size_t>> node_fqn_expected = {
    {"/ns/node", {0, 2, 3, 4, 5, 6, 9}},
    {"/node", {0, 1, 2}},
    {"/ns/foo/node", {0, 2, 4, 6, 7}},
    {"/ns/foo/a/node", {0, 2, 4, 6}},
    {"/ns/foo/a/bar/node", {0, 2, 4, 6, 8}},
    {"/ns/a/bar/node", {0, 2, 4, 6, 8}},
    {"/ns/foo/zoo/a/bar/node", {0, 2, 4, 6, 8}},
  };

  for (auto & kv : node_fqn_expected) {
    rclcpp::ParameterMap map = rclcpp::parameter_map_from(c_params, kv.first.c_str());
    const std::vector<rclcpp::Parameter> & params = map.at(kv.first);
    EXPECT_EQ(kv.second.size(), params.size());
    for (size_t i = 0; i < params.size(); ++i) {
      std::string param_name = "string_param" + std::to_string(kv.second[i]);
      EXPECT_STREQ(param_name.c_str(), params.at(i).get_name().c_str());
      EXPECT_STREQ(hello_world.c_str(), params.at(i).get_value<std::string>().c_str());
    }
  }

  for (size_t i = 0; i < node_names_keys.size(); ++i) {
    c_params->params[i].parameter_values[0].string_value = NULL;
  }
  delete[] c_hello_world;
  rcl_yaml_node_struct_fini(c_params);
}
