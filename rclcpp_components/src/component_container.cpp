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

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <thread>

#include "rcutils/logging_macros.h"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"
#include "rclcpp/utilities.hpp"

#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

enum class ExecutorType
{
  SingleThreaded,
  MultiThreaded,
  EventsCBG
};
inline std::optional<ExecutorType>
parse_executor_type(const std::string & arg)
{
  if (arg == "single-threaded") {return ExecutorType::SingleThreaded;}
  if (arg == "multi-threaded") {return ExecutorType::MultiThreaded;}
  if (arg == "events-cbg") {return ExecutorType::EventsCBG;}
  return std::nullopt;
}

inline std::string
executor_type_to_string(const ExecutorType & type)
{
  if (type == ExecutorType::SingleThreaded) {return "single-threaded";}
  if (type == ExecutorType::MultiThreaded) {return "multi-threaded";}
  if (type == ExecutorType::EventsCBG) {return "events-cbg";}
  return "unknown";
}
struct ParsedArgs
{
  ExecutorType executor_type = ExecutorType::SingleThreaded;
  bool isolated = false;
  bool help = false;
  bool invalid = false;
  std::string error_message;
};

inline ParsedArgs
parse_args(const std::vector<std::string> & args)
{
  ParsedArgs parsed;

  for (size_t i = 1; i < args.size(); ++i) {
    const std::string & arg = args[i];

    if (arg == "--isolated") {
      parsed.isolated = true;
    } else if (arg == "--help" || arg == "-h") {
      parsed.help = true;
      return parsed;
    } else if (arg == "--executor-type") {
      if (i + 1 >= args.size()) {
        parsed.invalid = true;
        parsed.error_message = "Missing value for --executor-type";
        return parsed;
      }
      auto option = parse_executor_type(args[++i]);
      if (!option) {
        parsed.invalid = true;
        parsed.error_message = "Invalid executor type: " + args[i];
        return parsed;
      }
      parsed.executor_type = option.value();
    }
  }

  return parsed;
}

inline std::shared_ptr<rclcpp::Executor>
make_executor(
  ExecutorType type,
  const rclcpp::ExecutorOptions & opts,
  int64_t num_threads)
{
  switch (type) {
    case ExecutorType::MultiThreaded:
      return std::make_shared<rclcpp::executors::MultiThreadedExecutor>(opts, num_threads);
    case ExecutorType::EventsCBG:
      return std::make_shared<rclcpp::executors::EventsCBGExecutor>(opts, num_threads);
    default:
      return std::make_shared<rclcpp::executors::SingleThreadedExecutor>(opts);
  }
}

void
print_usage()
{
  RCUTILS_LOG_INFO_NAMED(
    "component_container",
    "Usage: component_container [--executor-type <single-threaded|multi-threaded|events-cbg>]\n"
    "                           [--isolated]\n"
    "       component_container --help|-h\n"
    "Defaults: single-threaded, non-isolated\n"
    "          if multi-threaded: omitting thread_num means"
    " the executor will run with max available on system\n"
    "Examples: component_container --executor-type single-threaded\n"
    "          component_container --executor-type multi-threaded --ros_args -p thread_num:=4\n"
    "          component_container --executor-type events-cbg "
    "--isolated --ros_args -p thread_num:=1");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<std::string> arg_strs = rclcpp::remove_ros_arguments(argc, argv);
  auto args = parse_args(arg_strs);

  if (args.help) {
    print_usage();
    return 0;
  }

  if (args.invalid) {
    RCUTILS_LOG_ERROR_NAMED("component_container", "%s", args.error_message.c_str());
    print_usage();
    return 1;
  }

  std::shared_ptr<rclcpp::Executor> exec;
  std::shared_ptr<rclcpp_components::ComponentManager> node =
    std::make_shared<rclcpp_components::ComponentManager>();
  const int64_t num_threads = (node->has_parameter("thread_num")) ?
    node->get_parameter("thread_num").as_int() :
    std::thread::hardware_concurrency();
  std::string debug_msg;

  if (args.executor_type == ExecutorType::SingleThreaded &&
    num_threads > 0 &&
    num_threads != std::thread::hardware_concurrency())
  {
    RCUTILS_LOG_WARN_NAMED("component_container",
      "thread_num is not supported by the SingleThreadedExecutor. Ignoring...");
  }
  if (args.isolated) {
    // we use the ComponentManager node initially to get the `thread_num` parameter,
    // but temporarily delete it here before re-assigning it
    // if running with a ComponentManagerIsolated.
    // This is to avoid a possible race condition
    // where the 2 nodes may be briefly alive at the same time,
    node = nullptr;
    // The outer executor runs only the container manager's load/unload services.
    // Each loaded component gets its own dedicated executor of the requested type.
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    switch (args.executor_type) {
      case ExecutorType::MultiThreaded:
        node = std::make_shared<
          rclcpp_components::ComponentManagerIsolated<rclcpp::executors::MultiThreadedExecutor>>(
          rclcpp::ExecutorOptions(), num_threads);
        break;
      case ExecutorType::EventsCBG:
        node = std::make_shared<
          rclcpp_components::ComponentManagerIsolated<rclcpp::executors::EventsCBGExecutor>>(
          rclcpp::ExecutorOptions(), num_threads);
        break;
      default:
        node = std::make_shared<
          rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>>();
        break;
    }

    debug_msg = "Creating isolated component container with the following per-node settings: "
      "executor_type: " + executor_type_to_string(args.executor_type);
    if (args.executor_type != ExecutorType::SingleThreaded) {
      debug_msg += ", num_threads: " + std::to_string(num_threads);
    }
  } else {
    exec = make_executor(args.executor_type, rclcpp::ExecutorOptions(), num_threads);

    debug_msg = "Creating non-isolated component container with executor_type: " +
      executor_type_to_string(args.executor_type);
    if (args.executor_type != ExecutorType::SingleThreaded) {
      debug_msg += ", num_threads: " + std::to_string(num_threads);
    }
  }

  RCUTILS_LOG_DEBUG_NAMED("component_container", debug_msg.c_str());

  node->set_executor(exec);
  exec->add_node(node);
  exec->spin();

  return 0;
}
