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

#include "benchmark/benchmark.h"

#include <rcutils/logging.h>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_components/component_manager.hpp"

class ComponentTest : public benchmark::Fixture
{
public:
  ComponentTest()
  : component_manager_name("my_manager")
  {
  }

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#endif
  void SetUp(benchmark::State &) override
  {
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr, rclcpp::InitOptions().auto_initialize_logging(false));

    rclcpp::ExecutorOptions exec_options;
    exec_options.context = context;

    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(exec_options);

    manager = std::make_shared<rclcpp_components::ComponentManager>(
      executor, component_manager_name, rclcpp::NodeOptions().context(context));
    executor->add_node(manager);
  }

  void TearDown(benchmark::State &) override
  {
    context->shutdown("Test is complete");

    manager.reset();
    executor.reset();
    context.reset();
  }
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

  const std::string component_manager_name;

protected:
  rclcpp::Context::SharedPtr context;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
  std::shared_ptr<rclcpp_components::ComponentManager> manager;
};

BENCHMARK_F(ComponentTest, get_component_resources)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    std::vector<rclcpp_components::ComponentManager::ComponentResource> resources =
      manager->get_component_resources("rclcpp_components");
    if (resources.size() != 3) {
      state.SkipWithError("Wrong number of components found");
      break;
    }
  }
}

BENCHMARK_F(ComponentTest, create_component_factory)(benchmark::State & state)
{
  const std::vector<rclcpp_components::ComponentManager::ComponentResource> resources =
    manager->get_component_resources("rclcpp_components");
  if (resources.size() != 3) {
    state.SkipWithError("Wrong number of components found");
    return;
  }

  for (auto _ : state) {
    (void)_;
    manager->create_component_factory(resources[0]).reset();
  }
}

BENCHMARK_F(ComponentTest, create_node_instance)(benchmark::State & state)
{
  const std::vector<rclcpp_components::ComponentManager::ComponentResource> resources =
    manager->get_component_resources("rclcpp_components");
  if (resources.size() != 3) {
    state.SkipWithError("Wrong number of components found");
    return;
  }

  // Choosing resource 0 - the other two test components were shown empirically to yield
  // the same performance charactarisitics, so they shouldn't need their own benchmarks.
  const std::shared_ptr<rclcpp_components::NodeFactory> factory =
    manager->create_component_factory(resources[0]);

  const rclcpp::NodeOptions options = rclcpp::NodeOptions().context(context);

  for (auto _ : state) {
    (void)_;
    rclcpp_components::NodeInstanceWrapper node = factory->create_node_instance(options);
    benchmark::DoNotOptimize(node);
    benchmark::ClobberMemory();
  }
}
