// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"

#include "test_msgs/msg/empty.hpp"

#include "./executor_types.hpp"

template<typename T>
class TestIntraprocessExecutors : public ::testing::Test
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
    const auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::stringstream test_name;
    test_name << test_info->test_case_name() << "_" << test_info->name();
    node = std::make_shared<rclcpp::Node>("node", test_name.str());

    callback_count = 0u;

    const std::string topic_name = std::string("topic_") + test_name.str();

    rclcpp::PublisherOptions po;
    po.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    publisher = node->create_publisher<test_msgs::msg::Empty>(topic_name, rclcpp::QoS(1), po);

    auto callback = [this](test_msgs::msg::Empty::ConstSharedPtr) {
        this->callback_count.fetch_add(1u);
      };

    rclcpp::SubscriptionOptions so;
    so.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    subscription =
      node->create_subscription<test_msgs::msg::Empty>(
      topic_name, rclcpp::QoS(kNumMessages), std::move(callback), so);
  }

  void TearDown()
  {
    publisher.reset();
    subscription.reset();
    node.reset();
  }

  const size_t kNumMessages = 100;

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr publisher;
  rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr subscription;
  std::atomic_size_t callback_count;
};

TYPED_TEST_SUITE(TestIntraprocessExecutors, ExecutorTypes, ExecutorTypeNames);

TYPED_TEST(TestIntraprocessExecutors, testIntraprocessRetrigger) {
  // This tests that executors will continue to service intraprocess subscriptions in the case
  // that publishers aren't continuing to publish.
  // This was previously broken in that intraprocess guard conditions were only triggered on
  // publish and the test was added to prevent future regressions.
  static constexpr size_t kNumMessages = 100;

  using ExecutorType = TypeParam;
  ExecutorType executor;
  executor.add_node(this->node);

  EXPECT_EQ(0u, this->callback_count.load());
  this->publisher->publish(test_msgs::msg::Empty());

  // Wait for up to 5 seconds for the first message to come available.
  const std::chrono::milliseconds sleep_per_loop(10);
  int loops = 0;
  while (1u != this->callback_count.load() && loops < 500) {
    rclcpp::sleep_for(sleep_per_loop);
    executor.spin_some();
    loops++;
  }
  EXPECT_EQ(1u, this->callback_count.load());

  // reset counter
  this->callback_count.store(0u);

  for (size_t ii = 0; ii < kNumMessages; ++ii) {
    this->publisher->publish(test_msgs::msg::Empty());
  }

  // Fire a timer every 10ms up to 5 seconds waiting for subscriptions to be read.
  loops = 0;
  auto timer = this->node->create_wall_timer(
    std::chrono::milliseconds(10), [this, &executor, &loops]() {
      loops++;
      if (kNumMessages == this->callback_count.load() || loops == 500) {
        executor.cancel();
      }
    });
  executor.spin();
  EXPECT_EQ(kNumMessages, this->callback_count.load());
}
