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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class TestSubscription : public ::testing::Test
{
public:
  void OnMessage(const test_msgs::msg::Empty::SharedPtr msg)
  {
    (void)msg;
  }

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

protected:
  void initialize(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  {
    node = std::make_shared<rclcpp::Node>("test_subscription", "/ns", node_options);
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
};

struct TestParameters
{
  TestParameters(rclcpp::QoS qos, std::string description)
  : qos(qos), description(description) {}
  rclcpp::QoS qos;
  std::string description;
};

std::ostream & operator<<(std::ostream & out, const TestParameters & params)
{
  out << params.description;
  return out;
}

class TestSubscriptionInvalidIntraprocessQos
  : public TestSubscription,
  public ::testing::WithParamInterface<TestParameters>
{};

class TestSubscriptionSub : public ::testing::Test
{
public:
  void OnMessage(const test_msgs::msg::Empty::SharedPtr msg)
  {
    (void)msg;
  }

protected:
  static void SetUpTestCase()
  {
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_subscription", "/ns");
    subnode = node->create_sub_node("sub_ns");
  }

  void TearDown()
  {
    node.reset();
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr subnode;
};

class SubscriptionClassNodeInheritance : public rclcpp::Node
{
public:
  SubscriptionClassNodeInheritance()
  : Node("subscription_class_node_inheritance")
  {
  }

  void OnMessage(const test_msgs::msg::Empty::SharedPtr msg)
  {
    (void)msg;
  }

  void CreateSubscription()
  {
    auto callback = std::bind(
      &SubscriptionClassNodeInheritance::OnMessage, this, std::placeholders::_1);
    using test_msgs::msg::Empty;
    auto sub = this->create_subscription<Empty>("topic", 10, callback);
  }
};

class SubscriptionClass
{
public:
  void OnMessage(const test_msgs::msg::Empty::SharedPtr msg)
  {
    (void)msg;
  }

  void CreateSubscription()
  {
    auto node = std::make_shared<rclcpp::Node>("test_subscription_member_callback", "/ns");
    auto callback = std::bind(&SubscriptionClass::OnMessage, this, std::placeholders::_1);
    using test_msgs::msg::Empty;
    auto sub = node->create_subscription<Empty>("topic", 10, callback);
  }
};

class TestCftSubscription : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("test_cft_subscription", "/ns");
    rclcpp::SubscriptionOptionsBase options_base;
    options_base.content_filter_options.filter_expression =
      "int32_value = %0";
    options_base.content_filter_options.expression_parameters = {"10"};
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> subscription_options(
      options_base);
    auto callback = [](std::shared_ptr<const test_msgs::msg::BasicTypes>) {};
    sub = node->create_subscription<test_msgs::msg::BasicTypes>(
      "topic", 10, callback, subscription_options);
  }

  void TearDown()
  {
    sub.reset();
    node.reset();
  }

protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<test_msgs::msg::BasicTypes>::SharedPtr sub;
};

/*
   Testing subscription construction and destruction.
 */
TEST_F(TestSubscription, construction_and_destruction) {
  initialize();
  using test_msgs::msg::Empty;
  auto callback = [](const Empty::SharedPtr msg) {
      (void)msg;
    };
  {
    constexpr size_t depth = 10u;
    auto sub = node->create_subscription<Empty>("topic", depth, callback);

    EXPECT_NE(nullptr, sub->get_subscription_handle());
    // Converting to base class was necessary for the compiler to choose the const version of
    // get_subscription_handle()
    const rclcpp::SubscriptionBase * const_sub = sub.get();
    EXPECT_NE(nullptr, const_sub->get_subscription_handle());
    EXPECT_FALSE(sub->use_take_shared_method());

    EXPECT_NE(nullptr, sub->get_message_type_support_handle().typesupport_identifier);
    EXPECT_NE(nullptr, sub->get_message_type_support_handle().data);
    EXPECT_EQ(depth, sub->get_actual_qos().get_rmw_qos_profile().depth);
  }

  {
    ASSERT_THROW(
    {
      auto sub = node->create_subscription<Empty>("invalid_topic?", 10, callback);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}

/*
   Testing subscription construction and destruction for subnodes.
 */
TEST_F(TestSubscriptionSub, construction_and_destruction) {
  using test_msgs::msg::Empty;
  auto callback = [](const Empty::SharedPtr msg) {
      (void)msg;
    };
  {
    auto sub = subnode->create_subscription<Empty>("topic", 1, callback);
    EXPECT_STREQ(sub->get_topic_name(), "/ns/sub_ns/topic");
  }

  {
    auto sub = subnode->create_subscription<Empty>("/topic", 1, callback);
    EXPECT_STREQ(sub->get_topic_name(), "/topic");
  }

  {
    auto sub = subnode->create_subscription<Empty>("~/topic", 1, callback);
    std::string expected_topic_name =
      std::string(node->get_namespace()) + "/" + node->get_name() + "/topic";
    EXPECT_STREQ(sub->get_topic_name(), expected_topic_name.c_str());
  }

  {
    ASSERT_THROW(
    {
      auto sub = node->create_subscription<Empty>("invalid_topic?", 1, callback);
    }, rclcpp::exceptions::InvalidTopicNameError);
  }
}

/*
   Testing subscription creation signatures.
 */
TEST_F(TestSubscription, various_creation_signatures) {
  initialize();
  using test_msgs::msg::Empty;
  auto cb = [](test_msgs::msg::Empty::SharedPtr) {};
  {
    auto sub = node->create_subscription<Empty>("topic", 1, cb);
    (void)sub;
  }
  {
    auto sub = node->create_subscription<Empty>("topic", rclcpp::QoS(1), cb);
    (void)sub;
  }
  {
    auto sub =
      node->create_subscription<Empty>("topic", rclcpp::QoS(rclcpp::KeepLast(1)), cb);
    (void)sub;
  }
  {
    auto sub =
      node->create_subscription<Empty>("topic", rclcpp::QoS(rclcpp::KeepAll()), cb);
    (void)sub;
  }
  {
    auto sub = node->create_subscription<Empty>(
      "topic", 42, cb, rclcpp::SubscriptionOptions());
    (void)sub;
  }
  {
    auto sub = rclcpp::create_subscription<Empty>(
      node, "topic", 42, cb, rclcpp::SubscriptionOptions());
    (void)sub;
  }
  {
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
    options.allocator = std::make_shared<std::allocator<void>>();
    EXPECT_NE(nullptr, options.get_allocator());
    auto sub = rclcpp::create_subscription<Empty>(
      node, "topic", 42, cb, options);
    (void)sub;
  }
  {
    rclcpp::SubscriptionOptionsBase options_base;
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options(options_base);
    auto sub = rclcpp::create_subscription<Empty>(
      node, "topic", 42, cb, options);
    (void)sub;
  }
}

/*
   Testing subscriptions using std::bind.
 */
TEST_F(TestSubscription, callback_bind) {
  initialize();
  using test_msgs::msg::Empty;
  {
    // Member callback for plain class
    SubscriptionClass subscription_object;
    subscription_object.CreateSubscription();
  }
  {
    // Member callback for class inheriting from rclcpp::Node
    SubscriptionClassNodeInheritance subscription_object;
    subscription_object.CreateSubscription();
  }
  {
    // Member callback for class inheriting from testing::Test
    // Regression test for https://github.com/ros2/rclcpp/issues/479 where the TEST_F GTest macro
    // was interfering with rclcpp's `function_traits`.
    auto callback = std::bind(&TestSubscription::OnMessage, this, std::placeholders::_1);
    auto sub = node->create_subscription<Empty>("topic", 1, callback);
  }
}

/*
   Testing take.
 */
TEST_F(TestSubscription, take) {
  initialize();
  using test_msgs::msg::Empty;
  auto do_nothing = [](std::shared_ptr<const test_msgs::msg::Empty>) {FAIL();};
  {
    auto sub = node->create_subscription<test_msgs::msg::Empty>("~/test_take", 1, do_nothing);
    test_msgs::msg::Empty msg;
    rclcpp::MessageInfo msg_info;
    EXPECT_FALSE(sub->take(msg, msg_info));
  }
  {
    rclcpp::SubscriptionOptions so;
    so.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    auto sub = node->create_subscription<test_msgs::msg::Empty>("~/test_take", 1, do_nothing, so);
    rclcpp::PublisherOptions po;
    po.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    auto pub = node->create_publisher<test_msgs::msg::Empty>("~/test_take", 1, po);
    {
      test_msgs::msg::Empty msg;
      pub->publish(msg);
    }
    test_msgs::msg::Empty msg;
    rclcpp::MessageInfo msg_info;
    bool message_recieved = false;
    auto start = std::chrono::steady_clock::now();
    do {
      message_recieved = sub->take(msg, msg_info);
      std::this_thread::sleep_for(100ms);
    } while (!message_recieved && std::chrono::steady_clock::now() - start < 10s);
    EXPECT_TRUE(message_recieved);
  }
  // TODO(wjwwood): figure out a good way to test the intra-process exclusion behavior.
}

/*
   Testing take_serialized.
 */
TEST_F(TestSubscription, take_serialized) {
  initialize();
  using test_msgs::msg::Empty;
  auto do_nothing = [](std::shared_ptr<const rclcpp::SerializedMessage>) {FAIL();};
  {
    auto sub = node->create_subscription<test_msgs::msg::Empty>("~/test_take", 1, do_nothing);
    std::shared_ptr<rclcpp::SerializedMessage> msg = sub->create_serialized_message();
    rclcpp::MessageInfo msg_info;
    EXPECT_FALSE(sub->take_serialized(*msg, msg_info));
  }
  {
    rclcpp::SubscriptionOptions so;
    so.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    auto sub = node->create_subscription<test_msgs::msg::Empty>("~/test_take", 1, do_nothing, so);
    rclcpp::PublisherOptions po;
    po.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    auto pub = node->create_publisher<test_msgs::msg::Empty>("~/test_take", 1, po);
    {
      test_msgs::msg::Empty msg;
      pub->publish(msg);
    }
    std::shared_ptr<rclcpp::SerializedMessage> msg = sub->create_serialized_message();
    rclcpp::MessageInfo msg_info;
    bool message_recieved = false;
    auto start = std::chrono::steady_clock::now();
    do {
      message_recieved = sub->take_serialized(*msg, msg_info);
      std::this_thread::sleep_for(100ms);
    } while (!message_recieved && std::chrono::steady_clock::now() - start < 10s);
    EXPECT_TRUE(message_recieved);
  }
}

TEST_F(TestSubscription, rcl_subscription_init_error) {
  initialize();
  auto callback = [](std::shared_ptr<const test_msgs::msg::Empty>) {};
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_init, RCL_RET_TOPIC_NAME_INVALID);

  // reset() is not needed for triggering exception, just to avoid an unused return value warning
  EXPECT_THROW(
    node->create_subscription<test_msgs::msg::Empty>("topic", 10, callback).reset(),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestSubscription, rcl_subscription_fini_error) {
  initialize();
  auto callback = [](std::shared_ptr<const test_msgs::msg::Empty>) {};
  auto mock = mocking_utils::inject_on_return(
    "lib:rclcpp", rcl_subscription_fini, RCL_RET_ERROR);

  // Cleanup just fails, no exception expected
  EXPECT_NO_THROW(
    node->create_subscription<test_msgs::msg::Empty>("topic", 10, callback).reset());
}

TEST_F(TestSubscription, rcl_subscription_get_actual_qos_error) {
  initialize();
  auto callback = [](std::shared_ptr<const test_msgs::msg::Empty>) {};
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_get_actual_qos, nullptr);

  auto sub = node->create_subscription<test_msgs::msg::Empty>("topic", 10, callback);
  RCLCPP_EXPECT_THROW_EQ(
    sub->get_actual_qos(), std::runtime_error("failed to get qos settings: error not set"));
}

TEST_F(TestSubscription, rcl_take_type_erased_error) {
  initialize();
  auto callback = [](std::shared_ptr<const test_msgs::msg::Empty>) {};
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_take, RCL_RET_ERROR);

  auto sub = node->create_subscription<test_msgs::msg::Empty>("topic", 10, callback);
  test_msgs::msg::Empty msg;
  rclcpp::MessageInfo message_info;

  EXPECT_THROW(sub->take_type_erased(&msg, message_info), rclcpp::exceptions::RCLError);
}

TEST_F(TestSubscription, rcl_take_serialized_message_error) {
  initialize();
  auto callback = [](std::shared_ptr<const test_msgs::msg::Empty>) {};
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_take_serialized_message, RCL_RET_ERROR);

  auto sub = node->create_subscription<test_msgs::msg::Empty>("topic", 10, callback);
  rclcpp::SerializedMessage msg;
  rclcpp::MessageInfo message_info;

  EXPECT_THROW(sub->take_serialized(msg, message_info), rclcpp::exceptions::RCLError);
}

TEST_F(TestSubscription, rcl_subscription_get_publisher_count_error) {
  initialize();
  auto callback = [](std::shared_ptr<const test_msgs::msg::Empty>) {};
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_get_publisher_count, RCL_RET_ERROR);

  auto sub = node->create_subscription<test_msgs::msg::Empty>("topic", 10, callback);
  EXPECT_THROW(sub->get_publisher_count(), rclcpp::exceptions::RCLError);
}

TEST_F(TestSubscription, handle_loaned_message) {
  initialize();
  auto callback = [](std::shared_ptr<const test_msgs::msg::Empty>) {};
  auto sub = node->create_subscription<test_msgs::msg::Empty>("topic", 10, callback);

  test_msgs::msg::Empty msg;
  rclcpp::MessageInfo message_info;
  EXPECT_NO_THROW(sub->handle_loaned_message(&msg, message_info));
}

TEST_F(TestCftSubscription, is_cft_supported) {
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_subscription_is_cft_supported, false);
    EXPECT_FALSE(sub->is_cft_supported());
  }

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_subscription_is_cft_supported, true);
    EXPECT_TRUE(sub->is_cft_supported());
  }
}

TEST_F(TestCftSubscription, get_cft_expression_parameters_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_get_cft_expression_parameters, RCL_RET_ERROR);

  std::string filter_expression;
  std::vector<std::string> expression_parameters;
  EXPECT_THROW(
    sub->get_cft_expression_parameters(filter_expression, expression_parameters),
    rclcpp::exceptions::RCLError);
}

TEST_F(TestCftSubscription, set_cft_expression_parameters_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_set_cft_expression_parameters, RCL_RET_ERROR);

  std::string filter_expression = "int32_value = %0";
  std::string expression_parameter = "100";
  EXPECT_THROW(
    sub->set_cft_expression_parameters(filter_expression, {expression_parameter}),
    rclcpp::exceptions::RCLError);
}

/*
   Testing subscription with intraprocess enabled and invalid QoS
 */
TEST_P(TestSubscriptionInvalidIntraprocessQos, test_subscription_throws) {
  initialize(rclcpp::NodeOptions().use_intra_process_comms(true));
  rclcpp::QoS qos = GetParam().qos;
  using test_msgs::msg::Empty;
  {
    auto callback = std::bind(
      &TestSubscriptionInvalidIntraprocessQos::OnMessage,
      this,
      std::placeholders::_1);

    ASSERT_THROW(
      {auto subscription = node->create_subscription<Empty>(
          "topic",
          qos,
          callback);},
      std::invalid_argument);
  }
}

/*
   Testing subscription with invalid use_intra_process_comm
 */
TEST_P(TestSubscriptionInvalidIntraprocessQos, test_subscription_throws_intraprocess) {
  rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> options;
  options.use_intra_process_comm = static_cast<rclcpp::IntraProcessSetting>(5);

  initialize();
  rclcpp::QoS qos = GetParam().qos;
  auto callback = std::bind(
    &TestSubscriptionInvalidIntraprocessQos::OnMessage,
    this,
    std::placeholders::_1);

  RCLCPP_EXPECT_THROW_EQ(
    {auto subscription = node->create_subscription<test_msgs::msg::Empty>(
        "topic",
        qos,
        callback,
        options);},
    std::runtime_error("Unrecognized IntraProcessSetting value"));
}

static std::vector<TestParameters> invalid_qos_profiles()
{
  std::vector<TestParameters> parameters;

  parameters.reserve(3);
  parameters.push_back(
    TestParameters(
      rclcpp::QoS(rclcpp::KeepLast(10)).transient_local(),
      "transient_local_qos"));
  parameters.push_back(
    TestParameters(
      rclcpp::QoS(rclcpp::KeepAll()),
      "keep_all_qos"));

  return parameters;
}

INSTANTIATE_TEST_SUITE_P(
  TestSubscriptionThrows, TestSubscriptionInvalidIntraprocessQos,
  ::testing::ValuesIn(invalid_qos_profiles()),
  ::testing::PrintToStringParamName());
