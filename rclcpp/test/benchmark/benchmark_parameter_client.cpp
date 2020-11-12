// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "performance_test_fixture/performance_test_fixture.hpp"

#include "rclcpp/rclcpp.hpp"

class NodeParametersInterfaceTest : public performance_test_fixture::PerformanceTest
{
public:
  NodeParametersInterfaceTest()
  : node_name("my_node"),
    param_prefix("my_prefix"),
    param1_name(param_prefix + ".my_param_1"),
    param2_name(param_prefix + ".my_param_2"),
    param3_name(param_prefix + ".my_param_3")
  {
  }

  void SetUp(benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>(node_name);

    node->declare_parameter(param1_name);
    node->declare_parameter(param2_name);
    node->declare_parameter(param3_name);
    node->undeclare_parameter(param3_name);

    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void TearDown(benchmark::State & state)
  {
    performance_test_fixture::PerformanceTest::TearDown(state);

    node.reset();
    rclcpp::shutdown();
  }

  const std::string node_name;
  const std::string param_prefix;
  const std::string param1_name;
  const std::string param2_name;
  const std::string param3_name;

protected:
  rclcpp::Node::SharedPtr node;
};

BENCHMARK_F(NodeParametersInterfaceTest, declare_undeclare)(benchmark::State & state)
{
  for (auto _ : state) {
    node->declare_parameter(param3_name);
    node->undeclare_parameter(param3_name);
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, has_parameter_hit)(benchmark::State & state)
{
  for (auto _ : state) {
    if (!node->has_parameter(param1_name)) {
      state.SkipWithError("Parameter was expected");
      break;
    }
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, has_parameter_miss)(benchmark::State & state)
{
  for (auto _ : state) {
    if (node->has_parameter(param3_name)) {
      state.SkipWithError("Parameter was not expected");
      break;
    }
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, set_parameters_bool)(benchmark::State & state)
{
  const std::vector<rclcpp::Parameter> param_values1
  {
    rclcpp::Parameter(param1_name, true),
    rclcpp::Parameter(param2_name, false),
  };
  const std::vector<rclcpp::Parameter> param_values2
  {
    rclcpp::Parameter(param1_name, false),
    rclcpp::Parameter(param2_name, true),
  };

  reset_heap_counters();

  for (auto _ : state) {
    node->set_parameters(param_values2);
    node->set_parameters(param_values1);
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, set_parameters_atomically_bool)(benchmark::State & state)
{
  const std::vector<rclcpp::Parameter> param_values1
  {
    rclcpp::Parameter(param1_name, true),
    rclcpp::Parameter(param2_name, false),
  };
  const std::vector<rclcpp::Parameter> param_values2
  {
    rclcpp::Parameter(param1_name, false),
    rclcpp::Parameter(param2_name, true),
  };

  reset_heap_counters();

  for (auto _ : state) {
    node->set_parameters_atomically(param_values2);
    node->set_parameters_atomically(param_values1);
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, set_parameters_callback_bool)(benchmark::State & state)
{
  const std::vector<rclcpp::Parameter> param_values1
  {
    rclcpp::Parameter(param1_name, true),
    rclcpp::Parameter(param2_name, false),
  };
  const std::vector<rclcpp::Parameter> param_values2
  {
    rclcpp::Parameter(param1_name, false),
    rclcpp::Parameter(param2_name, true),
  };

  rcl_interfaces::msg::SetParametersResult callback_result;
  bool callback_received = false;
  callback_result.successful = true;
  auto callback =
    [&callback_result, &callback_received](const std::vector<rclcpp::Parameter> &) {
      callback_received = true;
      return callback_result;
    };
  auto handle = node->add_on_set_parameters_callback(callback);

  reset_heap_counters();

  for (auto _ : state) {
    node->set_parameters(param_values2);
    node->set_parameters(param_values1);
  }

  if (!callback_received) {
    state.SkipWithError("Callback is not functioning");
  }

  node->remove_on_set_parameters_callback(handle.get());
}

BENCHMARK_F(NodeParametersInterfaceTest, set_parameters_string)(benchmark::State & state)
{
  const std::vector<rclcpp::Parameter> param_values1
  {
    rclcpp::Parameter(param1_name, "param 1 value A"),
    rclcpp::Parameter(param2_name, "param 2 value B"),
  };
  const std::vector<rclcpp::Parameter> param_values2
  {
    rclcpp::Parameter(param1_name, "param 1 value B"),
    rclcpp::Parameter(param2_name, "param 2 value A"),
  };

  reset_heap_counters();

  for (auto _ : state) {
    node->set_parameters(param_values2);
    node->set_parameters(param_values1);
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, set_parameters_array)(benchmark::State & state)
{
  const std::vector<rclcpp::Parameter> param_values1
  {
    rclcpp::Parameter(param1_name, std::vector<int> {0, 1, 2}),
    rclcpp::Parameter(param2_name, std::vector<int> {3, 4, 5}),
  };
  const std::vector<rclcpp::Parameter> param_values2
  {
    rclcpp::Parameter(param1_name, std::vector<int> {4, 5, 6}),
    rclcpp::Parameter(param2_name, std::vector<int> {7, 8, 9}),
  };

  reset_heap_counters();

  for (auto _ : state) {
    node->set_parameters(param_values2);
    node->set_parameters(param_values1);
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, get_parameter)(benchmark::State & state)
{
  rclcpp::Parameter param1_value;

  reset_heap_counters();

  for (auto _ : state) {
    node->get_parameter(param1_name, param1_value);
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, list_parameters_hit)(benchmark::State & state)
{
  rcl_interfaces::msg::ListParametersResult param_list;
  const std::vector<std::string> prefixes
  {
    param_prefix,
  };

  reset_heap_counters();

  for (auto _ : state) {
    param_list = node->list_parameters(prefixes, 10);
    if (param_list.names.size() != 2) {
      state.SkipWithError("Expected node names");
      break;
    }
  }
}

BENCHMARK_F(NodeParametersInterfaceTest, list_parameters_miss)(benchmark::State & state)
{
  rcl_interfaces::msg::ListParametersResult param_list;
  const std::vector<std::string> prefixes
  {
    "your_param",
  };

  reset_heap_counters();

  for (auto _ : state) {
    param_list = node->list_parameters(prefixes, 10);
    if (param_list.names.size() != 0) {
      state.SkipWithError("Expected no node names");
      break;
    }
  }
}
