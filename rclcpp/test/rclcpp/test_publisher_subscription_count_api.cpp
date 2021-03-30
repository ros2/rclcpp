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

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_msgs/msg/empty.hpp"

namespace
{


template<typename ... Ts>
class NodeCreationPolicy
{
public:
  rclcpp::NodeOptions & node_options()
  {
    return options_;
  }

private:
  rclcpp::NodeOptions options_;
};

template<typename T, typename ... Ts>
class NodeCreationPolicy<T, Ts...>
{
public:
  NodeCreationPolicy()
  {
    gather<T, Ts...>(options_);
  }

  rclcpp::NodeOptions & node_options()
  {
    return options_;
  }

private:
  template<typename U>
  static rclcpp::NodeOptions &
  gather(rclcpp::NodeOptions & options)
  {
    return U::gather(options);
  }

  template<typename U, typename V, typename ... Ws>
  static rclcpp::NodeOptions &
  gather(rclcpp::NodeOptions & options)
  {
    return gather<V, Ws...>(U::gather(options));
  }

  rclcpp::NodeOptions options_;
};

template<bool value>
struct ShouldUseIntraprocess
{
  static rclcpp::NodeOptions & gather(rclcpp::NodeOptions & options)
  {
    return options.use_intra_process_comms(value);
  }
};

using UseIntraprocess = ShouldUseIntraprocess<true>;
using DoNotUseIntraprocess = ShouldUseIntraprocess<false>;

struct UseCustomContext
{
  static rclcpp::NodeOptions & gather(rclcpp::NodeOptions & options)
  {
    auto context = rclcpp::Context::make_shared();
    context->init(0, nullptr);
    return options.context(context);
  }
};

struct PrintTestDescription
{
  template<typename T>
  static std::string GetName(int i)
  {
    static_cast<void>(i);
    return T::description;
  }
};

}  // namespace


template<typename TestDescription>
class TestPublisherSubscriptionCount : public ::testing::Test
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

protected:
  static void OnMessage(test_msgs::msg::Empty::ConstSharedPtr msg)
  {
    (void)msg;
  }

  std::chrono::milliseconds offset{2000};
};

/* Testing publisher subscription count api and internal process subscription count.
 * Two subscriptions in the same topic, both using intraprocess comm.
 */
struct TwoSubscriptionsIntraprocessComm
{
  static constexpr const char * description =
    "two_subscriptions_intraprocess_comm";
  using FirstNodeCreationPolicy = NodeCreationPolicy<UseIntraprocess>;
  using SecondNodeCreationPolicy = NodeCreationPolicy<UseIntraprocess>;

  static constexpr bool first_node_talks_intraprocess{true};
  static constexpr bool both_nodes_talk_intraprocess{true};
};

/* Testing publisher subscription count api and internal process subscription count.
 * Two subscriptions, one using intra-process comm and the other not using it.
 */
struct TwoSubscriptionsOneIntraprocessOneNot
{
  static constexpr const char * description =
    "two_subscriptions_one_intraprocess_one_not";
  using FirstNodeCreationPolicy = NodeCreationPolicy<UseIntraprocess>;
  using SecondNodeCreationPolicy = NodeCreationPolicy<>;

  static constexpr bool first_node_talks_intraprocess{true};
  static constexpr bool both_nodes_talk_intraprocess{false};
};

/* Testing publisher subscription count api and internal process subscription count.
 * Two contexts, both using intra-process.
 */
struct TwoSubscriptionsInTwoContextsWithIntraprocessComm
{
  static constexpr const char * description =
    "two_subscriptions_in_two_contexts_with_intraprocess_comm";
  using FirstNodeCreationPolicy = NodeCreationPolicy<UseIntraprocess>;
  using SecondNodeCreationPolicy = NodeCreationPolicy<UseCustomContext, UseIntraprocess>;

  static constexpr bool first_node_talks_intraprocess{true};
  static constexpr bool both_nodes_talk_intraprocess{false};
};

/* Testing publisher subscription count api and internal process subscription count.
 * Two contexts, both of them not using intra-process comm.
 */
struct TwoSubscriptionsInTwoContextsWithoutIntraprocessComm
{
  static constexpr const char * description =
    "two_subscriptions_in_two_contexts_without_intraprocess_comm";
  using FirstNodeCreationPolicy = NodeCreationPolicy<>;
  using SecondNodeCreationPolicy = NodeCreationPolicy<UseCustomContext>;

  static constexpr bool first_node_talks_intraprocess{false};
  static constexpr bool both_nodes_talk_intraprocess{false};
};

using AllTestDescriptions = ::testing::Types<
  TwoSubscriptionsIntraprocessComm,
  TwoSubscriptionsOneIntraprocessOneNot,
  TwoSubscriptionsInTwoContextsWithIntraprocessComm,
  TwoSubscriptionsInTwoContextsWithoutIntraprocessComm
>;
TYPED_TEST_SUITE(TestPublisherSubscriptionCount, AllTestDescriptions, PrintTestDescription);


using test_msgs::msg::Empty;

TYPED_TEST(TestPublisherSubscriptionCount, increasing_and_decreasing_counts)
{
  using TestDescription = TypeParam;
  typename TestDescription::FirstNodeCreationPolicy my_node_creation_policy;
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
    "my_node",
    "/ns",
    my_node_creation_policy.node_options());
  auto publisher = node->create_publisher<Empty>("/topic", 10);

  EXPECT_EQ(publisher->get_subscription_count(), 0u);
  EXPECT_EQ(publisher->get_intra_process_subscription_count(), 0u);
  {
    auto sub = node->create_subscription<Empty>(
      "/topic", 10, &TestPublisherSubscriptionCount<TestDescription>::OnMessage);
    rclcpp::sleep_for(this->offset);
    EXPECT_EQ(publisher->get_subscription_count(), 1u);
    EXPECT_EQ(
      publisher->get_intra_process_subscription_count(),
      (TestDescription::first_node_talks_intraprocess ? 1u : 0u));
    {
      typename TestDescription::SecondNodeCreationPolicy another_node_creation_policy;
      rclcpp::Node::SharedPtr another_node = std::make_shared<rclcpp::Node>(
        "another_node",
        "/ns",
        another_node_creation_policy.node_options());
      auto another_sub = another_node->create_subscription<Empty>(
        "/topic", 10, &TestPublisherSubscriptionCount<TestDescription>::OnMessage);

      rclcpp::sleep_for(this->offset);
      EXPECT_EQ(publisher->get_subscription_count(), 2u);
      EXPECT_EQ(
        publisher->get_intra_process_subscription_count(),
        (TestDescription::first_node_talks_intraprocess ? 1u : 0u) +
        (TestDescription::both_nodes_talk_intraprocess ? 1u : 0u));
    }
    rclcpp::sleep_for(this->offset);
    EXPECT_EQ(publisher->get_subscription_count(), 1u);
    EXPECT_EQ(
      publisher->get_intra_process_subscription_count(),
      (TestDescription::first_node_talks_intraprocess ? 1u : 0u));
  }
  /**
    * Counts should be zero here, as all are subscriptions are out of scope.
    * Subscriptions count checking is always preceeded with an sleep, as random failures had been
    * detected without it. */
  rclcpp::sleep_for(this->offset);
  EXPECT_EQ(publisher->get_subscription_count(), 0u);
  EXPECT_EQ(publisher->get_intra_process_subscription_count(), 0u);
}
