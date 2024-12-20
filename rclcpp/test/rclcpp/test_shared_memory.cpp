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
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/basic_types.hpp"

using MessageT = test_msgs::msg::BasicTypes;
using rclcpp::experimental::executors::EventsExecutor;
using namespace std::chrono_literals;

/*
* The last reference to a loaned message has to return the loaned to the RMW
* when the message goes out of scope. This allows for memory to be
* reused for future loaned messages.
*
* Among the owners of the (shared) loaned messages we have:
* - User (storing messages beyond the subscription callback)
* - Subscriber (storing messages until subscription is executed)
* - Publisher (storing messages for late joiner subscriptions)
*
* This unit test intends to test:
* - Test that loaned are returned when expected, and check that memory is reused after return:
*   - User is the last owner
*   - Publisher is the last owner
*   - Subscriber is the last owner
* - Test that memory is indeed shared:
*   - User stores and overrides messages:
*     - Other subscription not yet executed (messages queued) should get
*       the modified version of messages.
*     - Publisher transient local messages should be affected:
*       - A late late joiner subscription should receive modified messages
* - Tests only to show current behaviour (which may be not the expected)
*   - Test max publisher depth setting (otherwise loaned are not used)
*   - Max loans that can be requested without publishing the messages
*   - Double delivery issue on transient local subscriptions
*
* Note: FastDDS requires using DYNAMIC_REUSABLE as history memory policy.
*/

class PubSubNode : public rclcpp::Node {
public:
  PubSubNode(std::string name, uint16_t pub_depth, uint16_t sub_depth, uint16_t user_depth)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false)), user_depth_(user_depth)
  {
    std::cout << "Created node: " <<  name << ": [pub_depth, sub_depth, user_depth] = ["
              << pub_depth << ", " << sub_depth << ", " << user_depth << "]" << std::endl;

    pub_ = this->create_publisher<MessageT>("topic", rclcpp::QoS(pub_depth).transient_local());
    sub_ = this->create_subscription<MessageT>("topic",
              rclcpp::QoS(sub_depth).transient_local(),
              [this](MessageT::SharedPtr msg) -> void
              {
                store_message(msg);
                //  std::stringstream ss;
                //  std::for_each(user_msgs_.begin(), user_msgs_.end(),
                //    [&ss](const auto &msg) { ss << msg->uint16_value << ", "; });
                // RCLCPP_INFO(this->get_logger(), "Subscription callback - User msgs: [%s]", ss.str().c_str());
              });
  }

  // Request N loaned messages and publish them
  void borrow_and_publish_loaned(uint16_t n = 1)
  {
    for (uint16_t i = 0; i < n; i++)
    {
      auto msg = pub_->borrow_loaned_message();
      loaned_is_reused(&msg.get());
      msg.get().uint16_value = count_++;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.get().uint16_value);
      pub_->publish(std::move(msg));
      std::this_thread::sleep_for(10ms);
    }
  }

  // Request N loaned messages but do not publish them, so they're not released
  void borrow_and_store_loaned_messages(uint32_t n)
  {
    for (uint16_t i = 0; i < n; i++) {
      auto loaned_msg = pub_->borrow_loaned_message();
      loaned_msgs_.emplace_back(std::move(loaned_msg));
    }
  }

  // Check if the requested loan is reusing memory from a previous loan
  bool loaned_is_reused(void * msg_ptr)
  {
      if (std::find(loaned_history_.begin(),
        loaned_history_.end(), msg_ptr) != loaned_history_.end())
      {
        memory_reused_ = true;
        return true;
      }
      loaned_history_.push_back(msg_ptr);
      return false;
  }

  // Copy the loaned messages to be stored beyond the subscriptions callback
  void store_message(MessageT::SharedPtr msg)
  {
    user_msgs_.push_back(msg);

    // But keep only the last N messages
    if (user_msgs_.size() > user_depth_) {
      user_msgs_.erase(user_msgs_.begin());
    }
  }

  // User overrides messages for testing purposes
  void user_override_messages(uint16_t val)
  {
    std::for_each(user_msgs_.begin(), user_msgs_.end(),
      [val](auto & msg) { msg->uint16_value = val; });

    // Print the overriden messages
    // std::stringstream ss;
    // std::for_each(user_msgs_.begin(), user_msgs_.end(),
    //   [&ss](const auto &msg) { ss << msg->uint16_value << ", "; });
    // RCLCPP_INFO(this->get_logger(), "Overriden messages: [%s]", ss.str().c_str());
  }

  bool memory_is_reused() { return memory_reused_; };

  uint16_t get_msg_value(uint16_t index)
  {
    return user_msgs_[index]->uint16_value;
  };

  size_t get_number_of_stored_messages()
  {
    return user_msgs_.size();
  };

  std::vector<MessageT::SharedPtr> get_user_messages()
  {
    return user_msgs_;
  }

  rclcpp::Publisher<MessageT>::SharedPtr pub_;
  rclcpp::Subscription<MessageT>::SharedPtr sub_;
  std::vector<MessageT::SharedPtr> user_msgs_;
  std::vector<rclcpp::LoanedMessage<MessageT>> loaned_msgs_;
  std::vector<void*> loaned_history_;
  uint16_t user_depth_;
  uint16_t count_{0};
  bool memory_reused_{false};
};


class TestSharedMemory : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    executor = std::make_unique<EventsExecutor>();
    executor_thread = std::thread([&]()
    {
      executor->spin();
    });
  }

  void TearDown()
  {
    executor->cancel();
    if (executor_thread.joinable())
    {
        executor_thread.join();
    }
    rclcpp::shutdown();
  }

  bool loaned_addresses_match(
      std::vector<MessageT::SharedPtr> loaned_msgs_1,
      std::vector<MessageT::SharedPtr> loaned_msgs_2)
  {
    if (loaned_msgs_1.size() != loaned_msgs_2.size()) {
      return false;
    }
    for (size_t i = 0; i < loaned_msgs_1.size(); ++i) {
      if (loaned_msgs_1[i].get() != loaned_msgs_2[i].get()) {
        return false;
      }
    }
    return true;
  }

  std::unique_ptr<EventsExecutor> executor;
  std::thread executor_thread;
};

// By controlling the depth of the publisher/subscription/user buffers, we can
// set the different scenarios for who is the last entity holding references
// to the messages, to test the right behaviour when last message goes out of scope.
TEST_F(TestSharedMemory, user_is_last_loan_owner)
{
  uint16_t user_depth = 10;
  auto pub_sub_node = std::make_shared<PubSubNode>("pub_sub_node", 1, 1, user_depth);
  auto idle_node = std::make_shared<PubSubNode>("idle_node", 1, 1 ,1);
  executor->add_node(pub_sub_node);

  // Publish 11 messages. User will delete oldest message (thus returning
  // the loan)
  pub_sub_node->borrow_and_publish_loaned(user_depth + 1);
  EXPECT_EQ(pub_sub_node->get_number_of_stored_messages(), user_depth);
  EXPECT_FALSE(pub_sub_node->memory_is_reused());
  EXPECT_EQ(pub_sub_node->get_msg_value(9), user_depth);

  // Publish 1 message more, memory should be reused since loan was returned
  pub_sub_node->borrow_and_publish_loaned(1);
  EXPECT_EQ(pub_sub_node->get_number_of_stored_messages(), user_depth);
  EXPECT_TRUE(pub_sub_node->memory_is_reused());
  EXPECT_EQ(pub_sub_node->get_msg_value(9), user_depth + 1);
}

TEST_F(TestSharedMemory, pub_is_last_loan_owner)
{
  uint16_t pub_depth = 10;
  auto pub_sub_node = std::make_shared<PubSubNode>("pub_sub_node", pub_depth, 1, 1);
  auto idle_node = std::make_shared<PubSubNode>("idle_node", 1, 1 ,1);
  executor->add_node(pub_sub_node);

  // Publish 11 messages. Publisher will delete oldest message (thus returning
  // the loan)
  pub_sub_node->borrow_and_publish_loaned(pub_depth + 1);
  EXPECT_EQ(pub_sub_node->get_number_of_stored_messages(), 1);
  EXPECT_FALSE(pub_sub_node->memory_is_reused());
  EXPECT_EQ(pub_sub_node->get_msg_value(0), pub_depth);

  // Publish 1 message more, memory should be reused since loan was returned
  pub_sub_node->borrow_and_publish_loaned(1);
  EXPECT_TRUE(pub_sub_node->memory_is_reused());
  EXPECT_EQ(pub_sub_node->get_msg_value(0), pub_depth + 1);
}

TEST_F(TestSharedMemory, sub_is_last_loan_owner)
{
  uint16_t sub_depth = 10;
  auto pub_sub_node = std::make_shared<PubSubNode>("pub_sub_node", 1, 1, 1);
  auto idle_node = std::make_shared<PubSubNode>("idle_node", 1, sub_depth, 1);
  executor->add_node(pub_sub_node);

  // Publish 11 messages. Idle subscription will delete oldest message (thus returning
  // the loan)
  pub_sub_node->borrow_and_publish_loaned(sub_depth + 1);
  EXPECT_EQ(pub_sub_node->get_number_of_stored_messages(), 1);
  EXPECT_FALSE(pub_sub_node->memory_is_reused());
  EXPECT_EQ(pub_sub_node->get_msg_value(0), sub_depth);

  // Publish 1 message more, memory should be reused since loan was returned
  pub_sub_node->borrow_and_publish_loaned(1);
  EXPECT_TRUE(pub_sub_node->memory_is_reused());
  EXPECT_EQ(pub_sub_node->get_msg_value(0), sub_depth + 1);
}

TEST_F(TestSharedMemory, memory_is_shared)
{
  // Publish 10 messages then overriden by the user.
  // Idle subscription is then executed: it should receive the modified messages.
  auto pub_sub_node = std::make_shared<PubSubNode>("pub_sub_node", 15, 1, 15);
  auto late_joiner_node = std::make_shared<PubSubNode>("late_joiner_node", 1, 10 , 15);
  executor->add_node(pub_sub_node);

  pub_sub_node->borrow_and_publish_loaned(10);
  pub_sub_node->user_override_messages(42);
  EXPECT_EQ(pub_sub_node->get_msg_value(9), 42);

  // Execute subscription which had the messages stored in its queue
  executor->add_node(late_joiner_node);
  std::this_thread::sleep_for(1s);

  EXPECT_EQ(late_joiner_node->get_number_of_stored_messages(), 10);
  // Verify it has the modified version
  EXPECT_EQ(late_joiner_node->get_msg_value(9), 42);

  auto pub_sub_loaned = pub_sub_node->get_user_messages();
  auto sub_loaned = late_joiner_node->get_user_messages();
  EXPECT_TRUE(loaned_addresses_match(pub_sub_loaned, sub_loaned));
}

// CycloneDDS suffers from the "double delivery issue" in which a late joiner subscription
// receives both interprocess & loaned messages, before segfaulting
TEST_F(TestSharedMemory, double_delivery)
{
  // Publish 10 messages and let the user override all of them.
  // Then a 2nd subscription (late joiner) is created, which should receive the modified messages.
  // CycloneDDS: The late joiner subscription first receives the 10 original messages,
  // (inter-process msgs?) and on the next publisher->publish, it receives the 10 loaned (modified) messages.
  // before a segfault.
  auto pub_sub_node = std::make_shared<PubSubNode>("pub_sub_node", 15, 1, 15);
  executor->add_node(pub_sub_node);

  pub_sub_node->borrow_and_publish_loaned(10);
  pub_sub_node->user_override_messages(42);

  auto late_joiner_node = std::make_shared<PubSubNode>("late_joiner_node", 1, 20 , 20);
  // Give some time for the subscription to receive transient local messages
  std::this_thread::sleep_for(50ms);
  executor->add_node(late_joiner_node);
  // Give some time for the executor to execute subscription and store the msgs
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(late_joiner_node->get_number_of_stored_messages(), 10);

  // If memory is shared, we should have obtained the modified messages
  EXPECT_EQ(late_joiner_node->get_msg_value(9), 42);

  // If memory is shared, both subscriptions should hold the same messages
  auto pub_sub_loaned = pub_sub_node->get_user_messages();
  auto late_sub_loaned = late_joiner_node->get_user_messages();

  // FastDDS: This fails
  EXPECT_TRUE(loaned_addresses_match(pub_sub_loaned, late_sub_loaned));

  // CycloneDDS: After this publish, the subscription receives the (modified)
  // loaned messages and program segfaults
  pub_sub_node->borrow_and_publish_loaned(1);
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(late_joiner_node->get_number_of_stored_messages(), 11);
  EXPECT_EQ(late_joiner_node->get_msg_value(10), 10);
}

/* Test limits:
// CycloneDDS: Show that publisher depth can't be more than 16, otherwise
// middleware refuses to loan messages (local allocator will be used)
TEST_F(TestSharedMemory, max_pub_depth)
{
  uint16_t max_depth = 16;
  auto pub_sub_node = std::make_shared<PubSubNode>("pub_sub_node", max_depth + 1 , 1, 2);
  executor->add_node(pub_sub_node);
  pub_sub_node->borrow_and_publish_loaned(2);
  EXPECT_EQ(pub_sub_node->get_number_of_stored_messages(), 2);
  EXPECT_FALSE(pub_sub_node->memory_is_reused());
}

// Depending on the publisher depth is the amount of loans we can make without throwing.
// FastDDS: throws after loaning the 3th msg and not sending it (if pub_depth = 1)
// CycloneDDS: throws after loaning the 9th msg and not sending it (if pub_depth = 1)
TEST_F(TestSharedMemory, max_loaned_stored)
{
  auto pub_sub_node = std::make_shared<PubSubNode>("pub_sub_node", 1, 1, 1);
  EXPECT_NO_THROW(pub_sub_node->borrow_and_store_loaned_messages(2));
  EXPECT_THROW(pub_sub_node->borrow_and_store_loaned_messages(1), rclcpp::exceptions::RCLError);
}
*/
