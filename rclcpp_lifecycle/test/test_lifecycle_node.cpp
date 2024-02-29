// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rcl_interfaces/srv/get_logger_levels.hpp"
#include "rcl_interfaces/srv/set_logger_levels.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "./mocking_utils/patch.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

using namespace std::chrono_literals;

static const std::chrono::nanoseconds DEFAULT_EVENT_TIMEOUT = std::chrono::seconds(3);
static const std::chrono::nanoseconds DEFAULT_EVENT_SLEEP_PERIOD = std::chrono::milliseconds(100);

static
bool wait_for_event(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  std::function<bool()> predicate,
  std::chrono::nanoseconds timeout,
  std::chrono::nanoseconds sleep_period)
{
  auto start = std::chrono::steady_clock::now();
  std::chrono::microseconds time_slept(0);

  bool predicate_result;
  while (!(predicate_result = predicate()) &&
    time_slept < std::chrono::duration_cast<std::chrono::microseconds>(timeout))
  {
    rclcpp::Event::SharedPtr graph_event = node->get_graph_event();
    node->wait_for_graph_change(graph_event, sleep_period);
    time_slept = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - start);
  }
  return predicate_result;
}

static
bool wait_for_topic(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & topic,
  std::chrono::nanoseconds timeout = DEFAULT_EVENT_TIMEOUT,
  std::chrono::nanoseconds sleep_period = DEFAULT_EVENT_SLEEP_PERIOD)
{
  return wait_for_event(
    node,
    [node, topic]()
    {
      auto topic_names_and_types = node->get_topic_names_and_types();
      return topic_names_and_types.end() != topic_names_and_types.find(topic);
    },
    timeout,
    sleep_period);
}

static
bool wait_for_service(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & service,
  std::chrono::nanoseconds timeout = DEFAULT_EVENT_TIMEOUT,
  std::chrono::nanoseconds sleep_period = DEFAULT_EVENT_SLEEP_PERIOD)
{
  return wait_for_event(
    node,
    [node, service]()
    {
      auto service_names_and_types = node->get_service_names_and_types();
      return service_names_and_types.end() != service_names_and_types.find(service);
    },
    timeout,
    sleep_period);
}

static
bool wait_for_service_by_node(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const std::string & node_name,
  const std::string & service,
  std::chrono::nanoseconds timeout = DEFAULT_EVENT_TIMEOUT,
  std::chrono::nanoseconds sleep_period = DEFAULT_EVENT_SLEEP_PERIOD)
{
  return wait_for_event(
    node,
    [node, node_name, service]()
    {
      auto service_names_and_types_by_node = node->get_service_names_and_types_by_node(
        node_name, "");
      return service_names_and_types_by_node.end() != service_names_and_types_by_node.find(
        service);
    },
    timeout,
    sleep_period);
}

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

struct GoodMood
{
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  static constexpr CallbackReturnT cb_ret = static_cast<CallbackReturnT>(
    lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS);
};
struct BadMood
{
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  static constexpr CallbackReturnT cb_ret = static_cast<CallbackReturnT>(
    lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE);
};

template<class Mood = GoodMood>
class MoodyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MoodyLifecycleNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(std::move(node_name))
  {}

  size_t number_of_callbacks = 0;

protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_CONFIGURING, get_current_state().id());
    ++number_of_callbacks;
    return Mood::cb_ret;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_ACTIVATING, get_current_state().id());
    ++number_of_callbacks;
    return Mood::cb_ret;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_DEACTIVATING, get_current_state().id());
    ++number_of_callbacks;
    return Mood::cb_ret;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_CLEANINGUP, get_current_state().id());
    ++number_of_callbacks;
    return Mood::cb_ret;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    EXPECT_EQ(State::TRANSITION_STATE_SHUTTINGDOWN, get_current_state().id());
    ++number_of_callbacks;
    return Mood::cb_ret;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &);
};

template<>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoodyLifecycleNode<GoodMood>::on_error(const rclcpp_lifecycle::State &)
{
  EXPECT_EQ(State::TRANSITION_STATE_ERRORPROCESSING, get_current_state().id());
  ADD_FAILURE();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template<>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoodyLifecycleNode<BadMood>::on_error(const rclcpp_lifecycle::State &)
{
  EXPECT_EQ(State::TRANSITION_STATE_ERRORPROCESSING, get_current_state().id());
  ++number_of_callbacks;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

TEST_F(TestDefaultStateMachine, empty_initializer) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  EXPECT_STREQ("testnode", test_node->get_name());
  EXPECT_STREQ("/", test_node->get_namespace());
  EXPECT_STREQ("/testnode", test_node->get_fully_qualified_name());
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
}

TEST_F(TestDefaultStateMachine, empty_initializer_rcl_errors) {
  {
    auto patch = mocking_utils::inject_on_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_state_machine_init, RCL_RET_ERROR);
    EXPECT_THROW(
      std::make_shared<EmptyLifecycleNode>("testnode").reset(),
      std::runtime_error);
  }
  {
    auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
    auto patch = mocking_utils::inject_on_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_state_machine_fini, RCL_RET_ERROR);
    EXPECT_NO_THROW(test_node.reset());
  }
}

TEST_F(TestDefaultStateMachine, check_logger_services_exist) {
  // Logger level services are disabled
  {
    rclcpp::NodeOptions options = rclcpp::NodeOptions();
    options.enable_logger_service(false);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "test_logger_service", "/test", options);
    auto get_client = node->create_client<rcl_interfaces::srv::GetLoggerLevels>(
      "/test/test_logger_service/get_logger_levels");
    ASSERT_FALSE(get_client->wait_for_service(2s));
    auto set_client = node->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      "/test/test_logger_service/set_logger_levels");
    ASSERT_FALSE(set_client->wait_for_service(2s));
  }
  // Logger level services are enabled
  {
    rclcpp::NodeOptions options = rclcpp::NodeOptions();
    options.enable_logger_service(true);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "test_logger_service", "/test", options);
    auto get_client = node->create_client<rcl_interfaces::srv::GetLoggerLevels>(
      "/test/test_logger_service/get_logger_levels");
    ASSERT_TRUE(get_client->wait_for_service(2s));
    auto set_client = node->create_client<rcl_interfaces::srv::SetLoggerLevels>(
      "/test/test_logger_service/set_logger_levels");
    ASSERT_TRUE(set_client->wait_for_service(2s));
  }
}

TEST_F(TestDefaultStateMachine, trigger_transition) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_ACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
  ASSERT_EQ(
    State::PRIMARY_STATE_FINALIZED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)).id());
}

TEST_F(TestDefaultStateMachine, trigger_transition_rcl_errors) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_state_machine_is_initialized, RCL_RET_ERROR);
    EXPECT_EQ(
      State::PRIMARY_STATE_UNCONFIGURED,
      test_node->trigger_transition(
        rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_trigger_transition_by_id, RCL_RET_ERROR);
    EXPECT_EQ(
      State::PRIMARY_STATE_UNCONFIGURED,
      test_node->trigger_transition(
        rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  }
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp_lifecycle", rcl_lifecycle_trigger_transition_by_label, RCL_RET_ERROR);
    EXPECT_EQ(
      State::TRANSITION_STATE_CONFIGURING,
      test_node->trigger_transition(
        rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  }
}

TEST_F(TestDefaultStateMachine, trigger_transition_with_error_code) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  auto success = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  auto reset_key = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  auto ret = reset_key;

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  test_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;

  test_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;

  test_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;

  test_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP), ret);
  ASSERT_EQ(success, ret);
  ret = reset_key;

  test_node->trigger_transition(
    rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN), ret);
  ASSERT_EQ(success, ret);
}

TEST_F(TestDefaultStateMachine, call_transitions_with_error_code) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  auto success = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  auto reset_key = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  auto ret = reset_key;

  test_node->configure(ret);

  EXPECT_EQ(success, ret);
  ret = reset_key;

  test_node->activate(ret);
  EXPECT_EQ(success, ret);
  ret = reset_key;

  test_node->deactivate(ret);
  EXPECT_EQ(success, ret);
  ret = reset_key;

  test_node->cleanup(ret);
  EXPECT_EQ(success, ret);
  ret = reset_key;

  test_node->shutdown(ret);
  EXPECT_EQ(success, ret);
}

TEST_F(TestDefaultStateMachine, call_transitions_without_code) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  auto configured = test_node->configure();
  EXPECT_EQ(configured.id(), State::PRIMARY_STATE_INACTIVE);

  auto activated = test_node->activate();
  EXPECT_EQ(activated.id(), State::PRIMARY_STATE_ACTIVE);

  auto deactivated = test_node->deactivate();
  EXPECT_EQ(deactivated.id(), State::PRIMARY_STATE_INACTIVE);

  auto unconfigured = test_node->cleanup();
  EXPECT_EQ(unconfigured.id(), State::PRIMARY_STATE_UNCONFIGURED);

  auto finalized = test_node->shutdown();
  EXPECT_EQ(finalized.id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(TestDefaultStateMachine, good_mood) {
  auto test_node = std::make_shared<MoodyLifecycleNode<GoodMood>>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  EXPECT_EQ(
    State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());
  EXPECT_EQ(
    State::PRIMARY_STATE_ACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_ACTIVATE)).id());
  EXPECT_EQ(
    State::PRIMARY_STATE_INACTIVE, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_DEACTIVATE)).id());
  EXPECT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CLEANUP)).id());
  EXPECT_EQ(
    State::PRIMARY_STATE_FINALIZED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)).id());

  // check if all callbacks were successfully overwritten
  EXPECT_EQ(5u, test_node->number_of_callbacks);
}

TEST_F(TestDefaultStateMachine, bad_mood) {
  auto test_node = std::make_shared<MoodyLifecycleNode<BadMood>>("testnode");

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
  EXPECT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED, test_node->trigger_transition(
      rclcpp_lifecycle::Transition(Transition::TRANSITION_CONFIGURE)).id());

  // check if all callbacks were successfully overwritten
  EXPECT_EQ(1u, test_node->number_of_callbacks);
}

TEST_F(TestDefaultStateMachine, lifecycle_subscriber) {
  auto test_node = std::make_shared<MoodyLifecycleNode<GoodMood>>("testnode");

  auto cb = [](const std::shared_ptr<const lifecycle_msgs::msg::State> msg) {(void) msg;};
  auto lifecycle_sub =
    test_node->create_subscription<lifecycle_msgs::msg::State>("~/empty", 10, cb);

  SUCCEED();
}

// Parameters are tested more thoroughly in rclcpp's test_node.cpp
// These are provided for coverage of lifecycle node's API
TEST_F(TestDefaultStateMachine, declare_parameters) {
  // "start_type_description_service" and "use_sim_time"
  const uint64_t builtin_param_count = 2;
  const uint64_t expected_param_count = 6 + builtin_param_count;
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  auto list_result = test_node->list_parameters({}, 0u);
  EXPECT_EQ(list_result.names.size(), builtin_param_count);
  EXPECT_STREQ(list_result.names[0].c_str(), "start_type_description_service");
  EXPECT_STREQ(list_result.names[1].c_str(), "use_sim_time");

  const std::string bool_name = "test_boolean";
  const std::string int_name = "test_int";

  // Default descriptor overload
  test_node->declare_parameter(bool_name, rclcpp::ParameterValue(false));

  // Explicit descriptor overload
  rcl_interfaces::msg::ParameterDescriptor int_descriptor;
  int_descriptor.name = int_name;
  int_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  int_descriptor.description = "Example integer parameter";
  test_node->declare_parameter(int_name, rclcpp::ParameterValue(42), int_descriptor);

  std::map<std::string, std::string> str_parameters;
  str_parameters["str_one"] = "stringy_string";
  str_parameters["str_two"] = "stringy_string_string";

  // Default descriptor overload
  test_node->declare_parameters("test_string", str_parameters);

  std::map<std::string,
    std::pair<double, rcl_interfaces::msg::ParameterDescriptor>> double_parameters;
  rcl_interfaces::msg::ParameterDescriptor double_descriptor_one;
  double_descriptor_one.name = "double_one";
  double_descriptor_one.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  double_parameters["double_one"] = std::make_pair(1.0, double_descriptor_one);

  rcl_interfaces::msg::ParameterDescriptor double_descriptor_two;
  double_descriptor_two.name = "double_two";
  double_descriptor_two.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  double_parameters["double_two"] = std::make_pair(2.0, double_descriptor_two);

  // Explicit descriptor overload
  test_node->declare_parameters("test_double", double_parameters);

  list_result = test_node->list_parameters({}, 0u);
  EXPECT_EQ(list_result.names.size(), expected_param_count);

  // The order of these names is not controlled by lifecycle_node, doing set equality
  std::set<std::string> expected_names = {
    "start_type_description_service",
    "test_boolean",
    "test_double.double_one",
    "test_double.double_two",
    "test_int",
    "test_string.str_one",
    "test_string.str_two",
    "use_sim_time",
  };
  std::set<std::string> actual_names(list_result.names.begin(), list_result.names.end());

  EXPECT_EQ(expected_names, actual_names);
}

TEST_F(TestDefaultStateMachine, check_parameters) {
  const uint64_t builtin_param_count = 2;
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  auto list_result = test_node->list_parameters({}, 0u);
  EXPECT_EQ(list_result.names.size(), builtin_param_count);
  EXPECT_STREQ(list_result.names[0].c_str(), "start_type_description_service");
  EXPECT_STREQ(list_result.names[1].c_str(), "use_sim_time");

  const std::string bool_name = "test_boolean";
  const std::string int_name = "test_int";
  std::vector<std::string> parameter_names = {bool_name, int_name};

  EXPECT_FALSE(test_node->has_parameter(bool_name));
  EXPECT_FALSE(test_node->has_parameter(int_name));
  EXPECT_THROW(
    test_node->get_parameters(parameter_names),
    rclcpp::exceptions::ParameterNotDeclaredException);

  // Default descriptor overload
  rcl_interfaces::msg::ParameterDescriptor bool_descriptor;
  bool_descriptor.dynamic_typing = true;
  test_node->declare_parameter(bool_name, rclcpp::ParameterValue(true), bool_descriptor);

  // Explicit descriptor overload
  rcl_interfaces::msg::ParameterDescriptor int_descriptor;
  int_descriptor.name = int_name;
  int_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  int_descriptor.description = "Example integer parameter";
  test_node->declare_parameter(int_name, rclcpp::ParameterValue(42), int_descriptor);

  // describe parameters
  auto descriptors = test_node->describe_parameters(parameter_names);
  EXPECT_EQ(descriptors.size(), parameter_names.size());

  // This actually throws inside NodeParameters::describe_parameters(), so it's not currently
  // possible to cover this method 100%.
  EXPECT_THROW(
    test_node->describe_parameter("not_a_real_parameter"),
    rclcpp::exceptions::ParameterNotDeclaredException);

  // describe parameter matches explicit descriptor
  auto descriptor = test_node->describe_parameter(int_name);
  EXPECT_STREQ(descriptor.name.c_str(), int_descriptor.name.c_str());
  EXPECT_EQ(descriptor.type, int_descriptor.type);
  EXPECT_STREQ(descriptor.description.c_str(), int_descriptor.description.c_str());

  // bool parameter exists and value matches
  EXPECT_TRUE(test_node->has_parameter(bool_name));
  EXPECT_EQ(test_node->get_parameter(bool_name).as_bool(), true);

  // int parameter exists and value matches
  EXPECT_TRUE(test_node->has_parameter(int_name));
  EXPECT_EQ(test_node->get_parameter(int_name).as_int(), 42);

  // Get multiple parameters at a time
  auto parameters = test_node->get_parameters(parameter_names);
  EXPECT_EQ(parameters.size(), parameter_names.size());
  EXPECT_EQ(parameters[0].as_bool(), true);
  EXPECT_EQ(parameters[1].as_int(), 42);

  // Get multiple parameters at a time with map
  std::map<std::string, rclcpp::ParameterValue> parameter_map;
  EXPECT_TRUE(test_node->get_parameters({}, parameter_map));

  EXPECT_EQ(parameter_map.size(), parameter_names.size() + builtin_param_count);

  // Check parameter types
  auto parameter_types = test_node->get_parameter_types(parameter_names);
  EXPECT_EQ(parameter_types.size(), parameter_names.size());
  EXPECT_EQ(parameter_types[0], rcl_interfaces::msg::ParameterType::PARAMETER_BOOL);
  EXPECT_EQ(parameter_types[1], rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);

  // Setting parameters
  size_t parameters_set = 0;
  auto callback = [&parameters_set](const std::vector<rclcpp::Parameter> & parameters) {
      parameters_set += parameters.size();
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    };

  // Hold callback handle. Callback is valid during the lifetime of this object.
  auto callback_handle = test_node->add_on_set_parameters_callback(callback);
  rclcpp::Parameter bool_parameter(bool_name, rclcpp::ParameterValue(false));
  EXPECT_TRUE(test_node->set_parameter(bool_parameter).successful);
  EXPECT_EQ(parameters_set, 1u);

  rclcpp::Parameter int_parameter(int_name, rclcpp::ParameterValue(7));
  test_node->set_parameters({int_parameter});
  EXPECT_EQ(parameters_set, 2u);

  test_node->remove_on_set_parameters_callback(callback_handle.get());
  rclcpp::Parameter bool_parameter2(bool_name, rclcpp::ParameterValue(true));
  EXPECT_TRUE(test_node->set_parameter(bool_parameter2).successful);
  EXPECT_EQ(parameters_set, 2u);


  // List parameters
  list_result = test_node->list_parameters({}, 0u);
  EXPECT_EQ(list_result.names.size(), parameter_names.size() + builtin_param_count);
  size_t index = 0;
  EXPECT_STREQ(list_result.names[index++].c_str(), "start_type_description_service");
  EXPECT_STREQ(list_result.names[index++].c_str(), parameter_names[0].c_str());
  EXPECT_STREQ(list_result.names[index++].c_str(), parameter_names[1].c_str());
  EXPECT_STREQ(list_result.names[index++].c_str(), "use_sim_time");

  // Undeclare parameter
  test_node->undeclare_parameter(bool_name);
  EXPECT_FALSE(test_node->has_parameter(bool_name));
  rclcpp::Parameter parameter;
  EXPECT_FALSE(test_node->get_parameter(bool_name, parameter));

  // Bool parameter has been undeclared, atomic setting should fail
  parameters = {
    rclcpp::Parameter(bool_name, rclcpp::ParameterValue(true)),
    rclcpp::Parameter(int_name, rclcpp::ParameterValue(0))};
  EXPECT_THROW(
    test_node->set_parameters_atomically(parameters),
    rclcpp::exceptions::ParameterNotDeclaredException);

  // Since setting parameters failed, this should remain the same
  EXPECT_EQ(test_node->get_parameter(int_name).as_int(), 7);

  // Bool parameter no longer exists, using "or" value
  EXPECT_FALSE(
    test_node->get_parameter_or(
      bool_name, parameter, rclcpp::Parameter(bool_name, rclcpp::ParameterValue(true))));
  EXPECT_TRUE(parameter.as_bool());
}

TEST_F(TestDefaultStateMachine, test_getters) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  auto options = test_node->get_node_options();
  EXPECT_EQ(0u, options.arguments().size());
  EXPECT_NE(nullptr, test_node->get_node_base_interface());
  EXPECT_NE(nullptr, test_node->get_node_clock_interface());
  EXPECT_NE(nullptr, test_node->get_node_graph_interface());
  EXPECT_NE(nullptr, test_node->get_node_logging_interface());
  EXPECT_NE(nullptr, test_node->get_node_time_source_interface());
  EXPECT_NE(nullptr, test_node->get_node_timers_interface());
  EXPECT_NE(nullptr, test_node->get_node_topics_interface());
  EXPECT_NE(nullptr, test_node->get_node_services_interface());
  EXPECT_NE(nullptr, test_node->get_node_parameters_interface());
  EXPECT_NE(nullptr, test_node->get_node_waitables_interface());
  EXPECT_NE(nullptr, test_node->get_graph_event());
  EXPECT_NE(nullptr, test_node->get_clock());
  EXPECT_LT(0u, test_node->now().nanoseconds());
  EXPECT_STREQ("testnode", test_node->get_logger().get_name());
  EXPECT_NE(nullptr, const_cast<const EmptyLifecycleNode *>(test_node.get())->get_clock());
  EXPECT_NE(nullptr, test_node->get_node_type_descriptions_interface());
}

TEST_F(TestDefaultStateMachine, test_graph_topics) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  auto names = test_node->get_node_names();

  ASSERT_NE(names.end(), std::find(names.begin(), names.end(), std::string("/testnode")));

  // Other topics may exist for an rclcpp::Node, but just checking the lifecycle one exists
  ASSERT_TRUE(wait_for_topic(test_node, "/testnode/transition_event"));
  auto topic_names_and_types = test_node->get_topic_names_and_types();
  EXPECT_STREQ(
    topic_names_and_types["/testnode/transition_event"][0].c_str(),
    "lifecycle_msgs/msg/TransitionEvent");

  EXPECT_EQ(1u, test_node->count_publishers("/testnode/transition_event"));
  EXPECT_EQ(0u, test_node->count_subscribers("/testnode/transition_event"));

  auto publishers_info = test_node->get_publishers_info_by_topic("/testnode/transition_event");
  EXPECT_EQ(1u, publishers_info.size());
  auto subscriptions_info =
    test_node->get_subscriptions_info_by_topic("/testnode/transition_event");
  EXPECT_EQ(0u, subscriptions_info.size());
}

TEST_F(TestDefaultStateMachine, test_graph_services) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  // These are specific to lifecycle nodes, other services are provided by rclcpp::Node
  ASSERT_TRUE(wait_for_service(test_node, "/testnode/change_state"));
  ASSERT_TRUE(wait_for_service(test_node, "/testnode/get_available_states"));
  ASSERT_TRUE(wait_for_service(test_node, "/testnode/get_available_transitions"));
  ASSERT_TRUE(wait_for_service(test_node, "/testnode/get_state"));
  ASSERT_TRUE(wait_for_service(test_node, "/testnode/get_transition_graph"));

  auto service_names_and_types = test_node->get_service_names_and_types();
  EXPECT_STREQ(
    service_names_and_types["/testnode/change_state"][0].c_str(),
    "lifecycle_msgs/srv/ChangeState");
  EXPECT_STREQ(
    service_names_and_types["/testnode/get_available_states"][0].c_str(),
    "lifecycle_msgs/srv/GetAvailableStates");
  EXPECT_STREQ(
    service_names_and_types["/testnode/get_available_transitions"][0].c_str(),
    "lifecycle_msgs/srv/GetAvailableTransitions");
  EXPECT_STREQ(
    service_names_and_types["/testnode/get_state"][0].c_str(),
    "lifecycle_msgs/srv/GetState");
  EXPECT_STREQ(
    service_names_and_types["/testnode/get_transition_graph"][0].c_str(),
    "lifecycle_msgs/srv/GetAvailableTransitions");
}

TEST_F(TestDefaultStateMachine, test_graph_services_by_node) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  // These are specific to lifecycle nodes, other services are provided by rclcpp::Node
  ASSERT_TRUE(wait_for_service_by_node(test_node, "testnode", "/testnode/change_state"));
  ASSERT_TRUE(wait_for_service_by_node(test_node, "testnode", "/testnode/get_available_states"));
  ASSERT_TRUE(
    wait_for_service_by_node(test_node, "testnode", "/testnode/get_available_transitions"));
  ASSERT_TRUE(wait_for_service_by_node(test_node, "testnode", "/testnode/get_state"));
  ASSERT_TRUE(wait_for_service_by_node(test_node, "testnode", "/testnode/get_transition_graph"));

  auto service_names_and_types_by_node =
    test_node->get_service_names_and_types_by_node("testnode", "");
  EXPECT_STREQ(
    service_names_and_types_by_node["/testnode/change_state"][0].c_str(),
    "lifecycle_msgs/srv/ChangeState");
  EXPECT_STREQ(
    service_names_and_types_by_node["/testnode/get_available_states"][0].c_str(),
    "lifecycle_msgs/srv/GetAvailableStates");
  EXPECT_STREQ(
    service_names_and_types_by_node["/testnode/get_available_transitions"][0].c_str(),
    "lifecycle_msgs/srv/GetAvailableTransitions");
  EXPECT_STREQ(
    service_names_and_types_by_node["/testnode/get_state"][0].c_str(),
    "lifecycle_msgs/srv/GetState");
  EXPECT_STREQ(
    service_names_and_types_by_node["/testnode/get_transition_graph"][0].c_str(),
    "lifecycle_msgs/srv/GetAvailableTransitions");
}

TEST_F(TestDefaultStateMachine, test_callback_groups) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  size_t num_groups = 0;
  test_node->for_each_callback_group(
    [&num_groups](rclcpp::CallbackGroup::SharedPtr group_ptr)
    {
      (void)group_ptr;
      num_groups++;
    });
  EXPECT_EQ(num_groups, 1u);

  auto group = test_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, true);
  EXPECT_NE(nullptr, group);

  num_groups = 0;
  test_node->for_each_callback_group(
    [&num_groups](rclcpp::CallbackGroup::SharedPtr group_ptr)
    {
      (void)group_ptr;
      num_groups++;
    });
  EXPECT_EQ(num_groups, 2u);
}

TEST_F(TestDefaultStateMachine, wait_for_graph_change)
{
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  EXPECT_THROW(
    test_node->wait_for_graph_change(nullptr, std::chrono::milliseconds(1)),
    rclcpp::exceptions::InvalidEventError);

  auto event = std::make_shared<rclcpp::Event>();
  EXPECT_THROW(
    test_node->wait_for_graph_change(event, std::chrono::milliseconds(0)),
    rclcpp::exceptions::EventNotRegisteredError);
}
