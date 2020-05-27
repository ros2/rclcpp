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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class TestDefaultStateMachine : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

class EmptyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EmptyLifecycleNode(std::string node_name)
  : rclcpp_lifecycle::LifecycleNode(std::move(node_name))
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
  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node->get_current_state().id());
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

  auto cb = [](const std::shared_ptr<lifecycle_msgs::msg::State> msg) {(void) msg;};
  auto lifecycle_sub =
    test_node->create_subscription<lifecycle_msgs::msg::State>("~/empty", 10, cb);

  SUCCEED();
}

// Parameters are tested more thoroughly in rclcpp's test_node.cpp
// These are provided for coverage of lifecycle node's API
TEST_F(TestDefaultStateMachine, declare_parameters) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  auto list_result = test_node->list_parameters({}, 0u);
  EXPECT_EQ(list_result.names.size(), 1u);
  EXPECT_STREQ(list_result.names[0].c_str(), "use_sim_time");

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
  EXPECT_EQ(list_result.names.size(), 7u);

  // The order of these names is not controlled by lifecycle_node, doing set equality
  std::set<std::string> expected_names = {
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
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");

  auto list_result = test_node->list_parameters({}, 0u);
  EXPECT_EQ(list_result.names.size(), 1u);
  EXPECT_STREQ(list_result.names[0].c_str(), "use_sim_time");

  const std::string bool_name = "test_boolean";
  const std::string int_name = "test_int";
  std::vector<std::string> parameter_names = {bool_name, int_name};

  EXPECT_FALSE(test_node->has_parameter(bool_name));
  EXPECT_FALSE(test_node->has_parameter(int_name));
  EXPECT_THROW(
    test_node->get_parameters(parameter_names),
    rclcpp::exceptions::ParameterNotDeclaredException);

  // Default descriptor overload
  test_node->declare_parameter(bool_name, rclcpp::ParameterValue(true));

  // Explicit descriptor overload
  rcl_interfaces::msg::ParameterDescriptor int_descriptor;
  int_descriptor.name = int_name;
  int_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  int_descriptor.description = "Example integer parameter";
  test_node->declare_parameter(int_name, rclcpp::ParameterValue(42), int_descriptor);

  // describe parameters
  auto descriptors = test_node->describe_parameters(parameter_names);
  EXPECT_EQ(descriptors.size(), parameter_names.size());

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

  // int param, bool param, and use_sim_time
  EXPECT_EQ(parameter_map.size(), 3u);

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

  test_node->add_on_set_parameters_callback(callback);
  rclcpp::Parameter bool_parameter(bool_name, rclcpp::ParameterValue(false));
  EXPECT_TRUE(test_node->set_parameter(bool_parameter).successful);
  EXPECT_EQ(parameters_set, 1u);

  rclcpp::Parameter int_parameter(int_name, rclcpp::ParameterValue(7));
  test_node->set_parameters({int_parameter});
  EXPECT_EQ(parameters_set, 2u);

  // List parameters
  list_result = test_node->list_parameters({}, 0u);
  EXPECT_EQ(list_result.names.size(), 3u);
  EXPECT_STREQ(list_result.names[0].c_str(), parameter_names[0].c_str());
  EXPECT_STREQ(list_result.names[1].c_str(), parameter_names[1].c_str());
  EXPECT_STREQ(list_result.names[2].c_str(), "use_sim_time");

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
}

TEST_F(TestDefaultStateMachine, test_graph) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  auto names = test_node->get_node_names();
  EXPECT_EQ(names.size(), 1u);
  EXPECT_STREQ(names[0].c_str(), "/testnode");

  // parameter_events, rosout, /testnode/transition_event
  auto topic_names_and_types = test_node->get_topic_names_and_types();
  EXPECT_EQ(topic_names_and_types.size(), 3u);
  EXPECT_STREQ(
    topic_names_and_types["/testnode/transition_event"][0].c_str(),
    "lifecycle_msgs/msg/TransitionEvent");

  auto service_names_and_types = test_node->get_service_names_and_types();
  EXPECT_EQ(service_names_and_types.size(), 11u);
  // These are specific to lifecycle nodes, other services are provided by rclcpp::Node
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

  EXPECT_EQ(1u, test_node->count_publishers("/testnode/transition_event"));
  EXPECT_EQ(0u, test_node->count_subscribers("/testnode/transition_event"));

  auto publishers_info = test_node->get_publishers_info_by_topic("/testnode/transition_event");
  EXPECT_EQ(publishers_info.size(), 1u);
  auto subscriptions_info =
    test_node->get_subscriptions_info_by_topic("/testnode/transition_event");
  EXPECT_EQ(subscriptions_info.size(), 0u);
}

TEST_F(TestDefaultStateMachine, test_callback_groups) {
  auto test_node = std::make_shared<EmptyLifecycleNode>("testnode");
  auto groups = test_node->get_callback_groups();
  EXPECT_EQ(groups.size(), 1u);

  auto group = test_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  EXPECT_NE(nullptr, group);

  groups = test_node->get_callback_groups();
  EXPECT_EQ(groups.size(), 2u);
  EXPECT_EQ(groups[1].lock().get(), group.get());
}
