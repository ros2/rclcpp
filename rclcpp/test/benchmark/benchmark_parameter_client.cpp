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
#include <vector>
#include <string>

#include "benchmark/benchmark.h"

#include "rclcpp/rclcpp.hpp"

class RemoteNodeTest : public benchmark::Fixture
{
public:
  RemoteNodeTest()
  : remote_node_name("my_remote_node")
  {
  }

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#endif
  void SetUp(benchmark::State &)
  {
    remote_context = std::make_shared<rclcpp::Context>();
    remote_context->init(0, nullptr, rclcpp::InitOptions().auto_initialize_logging(false));

    rclcpp::ExecutorOptions exec_options;
    exec_options.context = remote_context;

    remote_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exec_options);

    remote_node = std::make_shared<rclcpp::Node>(
      remote_node_name, rclcpp::NodeOptions().context(remote_context));
    remote_executor->add_node(remote_node);

    remote_thread = std::thread(&rclcpp::executors::SingleThreadedExecutor::spin, remote_executor);
  }

  void TearDown(benchmark::State &)
  {
    remote_executor->cancel();
    remote_context->shutdown("Test is complete");
    remote_thread.join();

    remote_node.reset();
    remote_executor.reset();
    remote_context.reset();
  }
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

  const std::string remote_node_name;

protected:
  rclcpp::Context::SharedPtr remote_context;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr remote_executor;
  rclcpp::Node::SharedPtr remote_node;
  std::thread remote_thread;
};

class ParameterClientTest : public RemoteNodeTest
{
public:
  ParameterClientTest()
  : node_name("my_node"),
    param_prefix("my_prefix"),
    param1_name(param_prefix + ".my_param_1"),
    param2_name(param_prefix + ".my_param_2"),
    param3_name(param_prefix + ".my_param_3")
  {
  }

  void SetUp(benchmark::State & state)
  {
    RemoteNodeTest::SetUp(state);

    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>(node_name);

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;

    remote_node->declare_parameter(
      param1_name, rclcpp::ParameterValue("param1_value"), descriptor);
    remote_node->declare_parameter(
      param2_name, rclcpp::ParameterValue(std::vector<int> {1, 2, 3}), descriptor);
    remote_node->declare_parameter(param3_name, rclcpp::ParameterValue{}, descriptor);
    remote_node->undeclare_parameter(param3_name);

    params_client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node_name);

    if (!params_client->wait_for_service()) {
      state.SkipWithError("Client failed to become ready");
    }
  }

  void TearDown(benchmark::State & state)
  {
    RemoteNodeTest::TearDown(state);

    rclcpp::shutdown();
    node.reset();
    params_client.reset();
  }

  const std::string node_name;
  const std::string param_prefix;
  const std::string param1_name;
  const std::string param2_name;
  const std::string param3_name;

protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::SyncParametersClient::SharedPtr params_client;
};

static bool result_is_successful(rcl_interfaces::msg::SetParametersResult result)
{
  return result.successful;
}

BENCHMARK_F(ParameterClientTest, create_destroy_client)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    params_client.reset();
    params_client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node_name);
    if (!params_client->wait_for_service()) {
      state.SkipWithError("Client failed to become ready");
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, has_parameter_hit)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    if (!params_client->has_parameter(param1_name)) {
      state.SkipWithError("Parameter was expected");
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, has_parameter_miss)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    if (params_client->has_parameter(param3_name)) {
      state.SkipWithError("Parameter was not expected");
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, set_parameters_bool)(benchmark::State & state)
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

  for (auto _ : state) {
    (void)_;
    std::vector<rcl_interfaces::msg::SetParametersResult> results =
      params_client->set_parameters(param_values2);
    if (!std::all_of(results.begin(), results.end(), result_is_successful)) {
      state.SkipWithError("Failed to set one or more parameters");
      break;
    }

    results = params_client->set_parameters(param_values1);
    if (!std::all_of(results.begin(), results.end(), result_is_successful)) {
      state.SkipWithError("Failed to set one or more parameters");
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, set_parameters_atomically_bool)(benchmark::State & state)
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

  for (auto _ : state) {
    (void)_;
    rcl_interfaces::msg::SetParametersResult result =
      params_client->set_parameters_atomically(param_values2);
    if (!result.successful) {
      state.SkipWithError(("Failed to set parameters: " + result.reason).c_str());
      break;
    }

    result = params_client->set_parameters_atomically(param_values1);
    if (!result.successful) {
      state.SkipWithError(("Failed to set parameters: " + result.reason).c_str());
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, set_parameters_string)(benchmark::State & state)
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

  for (auto _ : state) {
    (void)_;
    std::vector<rcl_interfaces::msg::SetParametersResult> results =
      params_client->set_parameters(param_values2);
    if (!std::all_of(results.begin(), results.end(), result_is_successful)) {
      state.SkipWithError("Failed to set one or more parameters");
      break;
    }

    results = params_client->set_parameters(param_values1);
    if (!std::all_of(results.begin(), results.end(), result_is_successful)) {
      state.SkipWithError("Failed to set one or more parameters");
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, set_parameters_array)(benchmark::State & state)
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

  for (auto _ : state) {
    (void)_;
    std::vector<rcl_interfaces::msg::SetParametersResult> results =
      params_client->set_parameters(param_values2);
    if (!std::all_of(results.begin(), results.end(), result_is_successful)) {
      state.SkipWithError("Failed to set one or more parameters");
      break;
    }

    results = params_client->set_parameters(param_values1);
    if (!std::all_of(results.begin(), results.end(), result_is_successful)) {
      state.SkipWithError("Failed to set one or more parameters");
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, get_parameters)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    std::vector<rclcpp::Parameter> results = params_client->get_parameters({param1_name});
    if (results.size() != 1 || results[0].get_name() != param1_name) {
      state.SkipWithError("Got the wrong parameter(s)");
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, list_parameters_hit)(benchmark::State & state)
{
  const std::vector<std::string> prefixes
  {
    param_prefix,
  };

  for (auto _ : state) {
    (void)_;
    rcl_interfaces::msg::ListParametersResult param_list =
      params_client->list_parameters(prefixes, 10);
    if (param_list.names.size() != 2) {
      state.SkipWithError("Expected parameters");
      break;
    }
  }
}

BENCHMARK_F(ParameterClientTest, list_parameters_miss)(benchmark::State & state)
{
  const std::vector<std::string> prefixes
  {
    "your_prefix",
  };

  for (auto _ : state) {
    (void)_;
    rcl_interfaces::msg::ListParametersResult param_list =
      params_client->list_parameters(prefixes, 10);
    if (param_list.names.size() != 0) {
      state.SkipWithError("Expected no parameters");
      break;
    }
  }
}
