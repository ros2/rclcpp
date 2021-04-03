// Copyright 2019-2020 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <memory>

#include "rclcpp/exceptions.hpp"
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
class TestSubscriptionPublisherCount : public ::testing::Test
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

struct OneContextPerTest
{
  static constexpr const char * description = "one_context_test";
  using NodeCreationPolicy = ::NodeCreationPolicy<>;
};

struct TwoContextsPerTest
{
  static constexpr const char * description = "two_contexts_test";
  using NodeCreationPolicy = ::NodeCreationPolicy<UseCustomContext>;
};

using AllTestDescriptions = ::testing::Types<OneContextPerTest, TwoContextsPerTest>;
TYPED_TEST_SUITE(TestSubscriptionPublisherCount, AllTestDescriptions, PrintTestDescription);


using test_msgs::msg::Empty;

TYPED_TEST(TestSubscriptionPublisherCount, increasing_and_decreasing_counts)
{
  using TestDescription = TypeParam;
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  auto subscription = node->create_subscription<Empty>(
    "/topic", 10, &TestSubscriptionPublisherCount<TestDescription>::OnMessage);

  EXPECT_EQ(subscription->get_publisher_count(), 0u);
  {
    auto pub = node->create_publisher<Empty>("/topic", 10);
    rclcpp::sleep_for(this->offset);
    EXPECT_EQ(subscription->get_publisher_count(), 1u);
    {
      typename TestDescription::NodeCreationPolicy node_creation_policy;
      rclcpp::Node::SharedPtr another_node = std::make_shared<rclcpp::Node>(
        "another_node",
        "/ns",
        node_creation_policy.node_options());
      auto another_pub =
        another_node->create_publisher<Empty>("/topic", 10);

      rclcpp::sleep_for(this->offset);
      EXPECT_EQ(subscription->get_publisher_count(), 2u);
    }
    rclcpp::sleep_for(this->offset);
    EXPECT_EQ(subscription->get_publisher_count(), 1u);
  }
  rclcpp::sleep_for(this->offset);
  EXPECT_EQ(subscription->get_publisher_count(), 0u);
}
