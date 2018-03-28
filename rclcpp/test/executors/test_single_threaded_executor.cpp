// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <memory>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

using namespace std::chrono_literals;

using rcl_interfaces::msg::IntraProcessMessage;

class TestSingleThreadedExecutor: public::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestSingleThreadedExecutor, timer_starvation) {
  std::string node_topic_name = "publisher";

  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("test_single_threaded_executor_timer_starvation");

  auto cbg = node->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);

  rclcpp::executors::SingleThreadedExecutor executor;

  //const size_t num_messages = 5 * std::min<size_t>(executor.get_number_of_threads(), 16);
  const size_t num_messages = 5;
  auto pub = node->create_publisher<IntraProcessMessage>(node_topic_name, num_messages);

  std::atomic_uint subscription_counter(0);
  auto sub_callback = [&subscription_counter](const IntraProcessMessage::SharedPtr /*msg*/)
  {
    ++subscription_counter;
    printf("Subscription callback %u\n", subscription_counter.load());
  };

  auto sub = node->create_subscription<IntraProcessMessage>(node_topic_name, num_messages,
    sub_callback,
    cbg);

  auto msg = std::make_shared<IntraProcessMessage>();
  pub->publish(msg);

  // wait a moment for everything to initialize
  //test_rclcpp::wait_for_subscriber(node, node_topic_name);

  // use atomic
  std::atomic_uint timer_counter(0);

  auto timer_callback =
  [&executor, &pub, &msg, &timer_counter, &subscription_counter, &num_messages](
    rclcpp::TimerBase & /*timer*/)
  {
    ++timer_counter;

    if (timer_counter.load() >= 10) {
      executor.cancel();
    }

    printf("Timer callback%u\n", timer_counter.load());
    pub->publish(msg);
    rclcpp::sleep_for(15ms);
  };

  std::vector<rclcpp::TimerBase::SharedPtr> timers;
  timers.push_back(node->create_wall_timer(10ms, timer_callback));
  executor.add_node(node);
  executor.spin();

  ASSERT_EQ(10, timer_counter);
  ASSERT_EQ(10, subscription_counter);
}

