// Copyright 2022 Open Source Robotics Foundation, Inc.
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
#include "rclcpp/wait_for_message.hpp"

#include "../mocking_utils/patch.hpp"
#include "../utils/rclcpp_gtest_macros.hpp"

#include "test_msgs/msg/basic_types.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestContentFilterSubscription, RMW_IMPLEMENTATION) : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_content_filter_node", "/ns");
    context = node->get_node_options().context();
    qos.reliable().transient_local();

    auto options = rclcpp::SubscriptionOptions();
    options.content_filter_options.filter_expression = filter_expression_init;
    options.content_filter_options.expression_parameters = expression_parameters_1;

    auto callback = [](std::shared_ptr<const test_msgs::msg::BasicTypes>) {};
    sub = node->create_subscription<test_msgs::msg::BasicTypes>(
      "content_filter_topic", qos, callback, options);
  }

  void TearDown()
  {
    sub.reset();
    node.reset();
  }

  template<typename Condition, typename Duration>
  bool wait_for(const Condition & condition, const Duration & timeout)
  {
    using clock = std::chrono::system_clock;
    auto start = clock::now();
    while (!condition()) {
      if ((clock::now() - start) > timeout) {
        return false;
      }
      rclcpp::spin_some(node);
    }
    return true;
  }

protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::Context::SharedPtr context;
  rclcpp::QoS qos{rclcpp::KeepLast{10}};
  rclcpp::Subscription<test_msgs::msg::BasicTypes>::SharedPtr sub;

  std::string filter_expression_init = "int32_value = %0";
  std::vector<std::string> expression_parameters_1 = {"3"};
  std::vector<std::string> expression_parameters_2 = {"4"};
};

bool operator==(const test_msgs::msg::BasicTypes & m1, const test_msgs::msg::BasicTypes & m2)
{
  return m1.bool_value == m2.bool_value &&
         m1.byte_value == m2.byte_value &&
         m1.char_value == m2.char_value &&
         m1.float32_value == m2.float32_value &&
         m1.float64_value == m2.float64_value &&
         m1.int8_value == m2.int8_value &&
         m1.uint8_value == m2.uint8_value &&
         m1.int16_value == m2.int16_value &&
         m1.uint16_value == m2.uint16_value &&
         m1.int32_value == m2.int32_value &&
         m1.uint32_value == m2.uint32_value &&
         m1.int64_value == m2.int64_value &&
         m1.uint64_value == m2.uint64_value;
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), is_cft_enabled) {
  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_subscription_is_cft_enabled, false);
    EXPECT_FALSE(sub->is_cft_enabled());
  }

  {
    auto mock = mocking_utils::patch_and_return(
      "lib:rclcpp", rcl_subscription_is_cft_enabled, true);
    EXPECT_TRUE(sub->is_cft_enabled());
  }
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), get_content_filter_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_get_content_filter, RCL_RET_ERROR);

  rclcpp::ContentFilterOptions options;
  EXPECT_THROW(
    options = sub->get_content_filter(),
    rclcpp::exceptions::RCLError);
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), set_content_filter_error) {
  auto mock = mocking_utils::patch_and_return(
    "lib:rclcpp", rcl_subscription_set_content_filter, RCL_RET_ERROR);

  std::string filter_expression = "int32_value = %0";
  std::string expression_parameter = "100";
  EXPECT_THROW(
    sub->set_content_filter(filter_expression, {expression_parameter}),
    rclcpp::exceptions::RCLError);
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), get_content_filter) {
  rclcpp::ContentFilterOptions options;

  if (sub->is_cft_enabled()) {
    EXPECT_NO_THROW(
      options = sub->get_content_filter());

    EXPECT_EQ(options.filter_expression, filter_expression_init);
    EXPECT_EQ(options.expression_parameters, expression_parameters_1);
  } else {
    EXPECT_THROW(
      options = sub->get_content_filter(),
      rclcpp::exceptions::RCLError);
  }
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), set_content_filter) {
  if (sub->is_cft_enabled()) {
    EXPECT_NO_THROW(
      sub->set_content_filter(filter_expression_init, expression_parameters_2));
  } else {
    EXPECT_THROW(
      sub->set_content_filter(filter_expression_init, expression_parameters_2),
      rclcpp::exceptions::RCLError);
  }
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), content_filter_get_begin) {
  using namespace std::chrono_literals;
  {
    test_msgs::msg::BasicTypes msg;
    rclcpp::MessageInfo msg_info;
    EXPECT_FALSE(sub->take(msg, msg_info));
  }
  {
    rclcpp::PublisherOptions po;
    auto pub = node->create_publisher<test_msgs::msg::BasicTypes>("content_filter_topic", qos, po);

    auto connected = [pub, sub = this->sub]() -> bool {
        return pub->get_subscription_count() && sub->get_publisher_count();
      };
    ASSERT_TRUE(wait_for(connected, 10s));

    test_msgs::msg::BasicTypes original_message;
    original_message.int32_value = 3;
    pub->publish(original_message);

    test_msgs::msg::BasicTypes output_message;
    bool receive = wait_for_message(output_message, sub, context, 10s);
    EXPECT_TRUE(receive);
    EXPECT_EQ(original_message, output_message);

    if (sub->is_cft_enabled()) {
      EXPECT_NO_THROW(
        sub->set_content_filter(filter_expression_init, expression_parameters_2));
      // waiting to allow for filter propagation
      std::this_thread::sleep_for(std::chrono::seconds(10));

      test_msgs::msg::BasicTypes original_message;
      original_message.int32_value = 3;
      pub->publish(original_message);

      test_msgs::msg::BasicTypes output_message;
      bool receive = wait_for_message(output_message, sub, context, 10s);
      EXPECT_FALSE(receive);
    }
  }
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), content_filter_get_later) {
  using namespace std::chrono_literals;
  {
    test_msgs::msg::BasicTypes msg;
    rclcpp::MessageInfo msg_info;
    EXPECT_FALSE(sub->take(msg, msg_info));
  }
  {
    rclcpp::PublisherOptions po;
    auto pub = node->create_publisher<test_msgs::msg::BasicTypes>("content_filter_topic", qos, po);

    auto connected = [pub, sub = this->sub]() -> bool {
        return pub->get_subscription_count() && sub->get_publisher_count();
      };
    ASSERT_TRUE(wait_for(connected, 10s));

    test_msgs::msg::BasicTypes original_message;
    original_message.int32_value = 4;
    pub->publish(original_message);

    test_msgs::msg::BasicTypes output_message;
    bool receive = wait_for_message(output_message, sub, context, 10s);
    if (sub->is_cft_enabled()) {
      EXPECT_FALSE(receive);
    } else {
      EXPECT_TRUE(receive);
      EXPECT_EQ(original_message, output_message);
    }

    if (sub->is_cft_enabled()) {
      EXPECT_NO_THROW(
        sub->set_content_filter(filter_expression_init, expression_parameters_2));
      // waiting to allow for filter propagation
      std::this_thread::sleep_for(std::chrono::seconds(10));

      test_msgs::msg::BasicTypes original_message;
      original_message.int32_value = 4;
      pub->publish(original_message);

      test_msgs::msg::BasicTypes output_message;
      bool receive = wait_for_message(output_message, sub, context, 10s);
      EXPECT_TRUE(receive);
      EXPECT_EQ(original_message, output_message);
    }
  }
}

TEST_F(CLASSNAME(TestContentFilterSubscription, RMW_IMPLEMENTATION), content_filter_reset) {
  using namespace std::chrono_literals;
  {
    test_msgs::msg::BasicTypes msg;
    rclcpp::MessageInfo msg_info;
    EXPECT_FALSE(sub->take(msg, msg_info));
  }
  {
    rclcpp::PublisherOptions po;
    auto pub = node->create_publisher<test_msgs::msg::BasicTypes>("content_filter_topic", qos, po);

    auto connected = [pub, sub = this->sub]() -> bool {
        return pub->get_subscription_count() && sub->get_publisher_count();
      };
    ASSERT_TRUE(wait_for(connected, 10s));

    test_msgs::msg::BasicTypes original_message;
    original_message.int32_value = 4;
    pub->publish(original_message);

    test_msgs::msg::BasicTypes output_message;
    bool receive = wait_for_message(output_message, sub, context, 10s);
    if (sub->is_cft_enabled()) {
      EXPECT_FALSE(receive);
    } else {
      EXPECT_TRUE(receive);
      EXPECT_EQ(original_message, output_message);
    }

    if (sub->is_cft_enabled()) {
      EXPECT_NO_THROW(
        sub->set_content_filter(""));
      // waiting to allow for filter propagation
      std::this_thread::sleep_for(std::chrono::seconds(10));

      test_msgs::msg::BasicTypes original_message;
      original_message.int32_value = 4;
      pub->publish(original_message);

      test_msgs::msg::BasicTypes output_message;
      bool receive = wait_for_message(output_message, sub, context, 10s);
      EXPECT_TRUE(receive);
      EXPECT_EQ(original_message, output_message);
    }
  }
}

TEST_F(
  CLASSNAME(
    TestContentFilterSubscription,
    RMW_IMPLEMENTATION), create_two_content_filters_with_same_topic_name_and_destroy) {

  // Create another content filter
  auto options = rclcpp::SubscriptionOptions();

  std::string filter_expression = "int32_value > %0";
  std::vector<std::string> expression_parameters = {"4"};

  options.content_filter_options.filter_expression = filter_expression;
  options.content_filter_options.expression_parameters = expression_parameters;

  auto callback = [](std::shared_ptr<const test_msgs::msg::BasicTypes>) {};
  auto sub_2 = node->create_subscription<test_msgs::msg::BasicTypes>(
    "content_filter_topic", qos, callback, options);

  EXPECT_NE(nullptr, sub_2);
  sub_2.reset();
}
