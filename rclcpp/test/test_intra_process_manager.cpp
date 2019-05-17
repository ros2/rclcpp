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

#include <memory>
#include <string>
#include <utility>

#define RCLCPP_BUILDING_LIBRARY 1
#include "gtest/gtest.h"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/mapped_ring_buffer.hpp"
#include "rmw/types.h"

// Mock up publisher and subscription base to avoid needing an rmw impl.
namespace rclcpp
{
namespace mock
{

class PublisherBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase)

  PublisherBase()
  : mock_topic_name(""), mock_queue_size(0) {}

  virtual ~PublisherBase()
  {}

  const char * get_topic_name() const
  {
    return mock_topic_name.c_str();
  }
  size_t get_queue_size() const
  {
    return mock_queue_size;
  }

  bool
  operator==(const rmw_gid_t * gid) const
  {
    (void)gid;
    return false;
  }

  virtual
  mapped_ring_buffer::MappedRingBufferBase::SharedPtr
  make_mapped_ring_buffer(size_t size) const
  {
    (void)size;
    return nullptr;
  }

  std::string mock_topic_name;
  size_t mock_queue_size;
};

template<typename T, typename Alloc = std::allocator<void>>
class Publisher : public PublisherBase
{
public:
  using MessageAllocTraits = allocator::AllocRebind<T, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, T>;
  using MessageUniquePtr = std::unique_ptr<T, MessageDeleter>;
  std::shared_ptr<MessageAlloc> allocator_;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<T, Alloc>)

  Publisher()
  {
    allocator_ = std::make_shared<MessageAlloc>();
  }

  mapped_ring_buffer::MappedRingBufferBase::SharedPtr
  make_mapped_ring_buffer(size_t size) const override
  {
    return mapped_ring_buffer::MappedRingBuffer<
      T,
      typename Publisher<T, Alloc>::MessageAlloc
    >::make_shared(size, allocator_);
  }

  std::shared_ptr<MessageAlloc> get_allocator()
  {
    return allocator_;
  }
};

}  // namespace mock
}  // namespace rclcpp

namespace rclcpp
{
namespace mock
{

class SubscriptionBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionBase)

  SubscriptionBase()
  : mock_topic_name(""), mock_queue_size(0) {}

  const char * get_topic_name() const
  {
    return mock_topic_name.c_str();
  }
  size_t get_queue_size() const
  {
    return mock_queue_size;
  }

  std::string mock_topic_name;
  size_t mock_queue_size;
};

}  // namespace mock
}  // namespace rclcpp

// Prevent rclcpp/publisher_base.hpp and rclcpp/subscription.hpp from being imported.
#define RCLCPP__PUBLISHER_BASE_HPP_
#define RCLCPP__SUBSCRIPTION_BASE_HPP_
// Force ipm to use our mock publisher class.
#define Publisher mock::Publisher
#define PublisherBase mock::PublisherBase
#define SubscriptionBase mock::SubscriptionBase
#include "../src/rclcpp/intra_process_manager.cpp"
#include "../src/rclcpp/intra_process_manager_impl.cpp"
#undef SubscriptionBase
#undef Publisher
#undef PublisherBase

// NOLINTNEXTLINE(build/include_order)
#include <rcl_interfaces/msg/intra_process_message.hpp>

/*
   This tests the "normal" usage of the class:
   - Creates two publishers on different topics.
   - Creates a subscription which matches one of them.
   - Publishes on each publisher with different message content.
   - Try's to take the message from the non-matching publish, should fail.
   - Try's to take the message from the matching publish, should work.
   - Asserts the message it got back was the one that went in (since there's only one subscription).
   - Try's to take the message again, should fail.
 */
TEST(TestIntraProcessManager, nominal) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto p1 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p1->mock_topic_name = "nominal1";
  p1->mock_queue_size = 2;

  auto p2 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p2->mock_topic_name = "nominal2";
  p2->mock_queue_size = 10;

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto p1_id = ipm.add_publisher(p1);
  auto p2_id = ipm.add_publisher(p2);
  auto s1_id = ipm.add_subscription(s1);

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  auto p1_m1_original_address = unique_msg.get();
  auto p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm_msg->message_sequence = 43;
  ipm_msg->publisher_id = 43;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  auto p2_m1_id = ipm.store_intra_process_message(p2_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm.take_intra_process_message(p2_id, p2_m1_id, s1_id, unique_msg);
  EXPECT_EQ(nullptr, unique_msg);  // Should fail since p2 and s1 don't have the same topic.
  unique_msg.reset();

  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    EXPECT_EQ(p1_m1_original_address, unique_msg.get());
  }

  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_EQ(nullptr, unique_msg);  // Should fail, since the message was already taken.

  ipm_msg->message_sequence = 44;
  ipm_msg->publisher_id = 44;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm_msg->message_sequence = 45;
  ipm_msg->publisher_id = 45;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm_msg->message_sequence = 46;
  ipm_msg->publisher_id = 46;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);
}

/*
   Simulates the case where a publisher is removed between publishing and the matching take.
   - Creates a publisher and subscription on the same topic.
   - Publishes a message.
   - Remove the publisher.
   - Try's to take the message, should fail since the publisher (and its storage) is gone.
 */
TEST(TestIntraProcessManager, remove_publisher_before_trying_to_take) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto p1 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p1->mock_topic_name = "nominal1";
  p1->mock_queue_size = 10;

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto p1_id = ipm.add_publisher(p1);
  auto s1_id = ipm.add_subscription(s1);

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  auto p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm.remove_publisher(p1_id);

  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_EQ(nullptr, unique_msg);  // Should fail, since the publisher is gone.
}

/*
   Tests whether or not removed subscriptions affect take behavior.
   - Creates a publisher and three subscriptions on the same topic.
   - Publish a message, keep the original point for later comparison.
   - Take with one subscription, should work.
   - Remove a different subscription.
   - Take with the final subscription, should work.
   - Assert the previous take returned ownership of the original object published.
 */
TEST(TestIntraProcessManager, removed_subscription_affects_take) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto p1 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p1->mock_topic_name = "nominal1";
  p1->mock_queue_size = 10;

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto s2 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s2->mock_topic_name = "nominal1";
  s2->mock_queue_size = 10;

  auto s3 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s3->mock_topic_name = "nominal1";
  s3->mock_queue_size = 10;

  auto p1_id = ipm.add_publisher(p1);
  auto s1_id = ipm.add_subscription(s1);
  auto s2_id = ipm.add_subscription(s2);
  auto s3_id = ipm.add_subscription(s3);

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  auto original_message_pointer = unique_msg.get();
  auto p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer, unique_msg.get());
  }
  unique_msg.reset();

  ipm.remove_subscription(s2_id);

  // Take using s3, the remaining subscription.
  ipm.take_intra_process_message(p1_id, p1_m1_id, s3_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    // Should match the original pointer since s2 was removed first.
    EXPECT_EQ(original_message_pointer, unique_msg.get());
  }

  // Take using s2, should fail since s2 was removed.
  unique_msg.reset();
  ipm.take_intra_process_message(p1_id, p1_m1_id, s2_id, unique_msg);
  EXPECT_EQ(nullptr, unique_msg);
}

/*
   This tests normal operation with multiple subscriptions and one publisher.
   - Creates a publisher and three subscriptions on the same topic.
   - Publish a message.
   - Take with each subscription, checking that the last takes the original back.
 */
TEST(TestIntraProcessManager, multiple_subscriptions_one_publisher) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto p1 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p1->mock_topic_name = "nominal1";
  p1->mock_queue_size = 10;

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto s2 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s2->mock_topic_name = "nominal1";
  s2->mock_queue_size = 10;

  auto s3 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s3->mock_topic_name = "nominal1";
  s3->mock_queue_size = 10;

  auto p1_id = ipm.add_publisher(p1);
  auto s1_id = ipm.add_subscription(s1);
  auto s2_id = ipm.add_subscription(s2);
  auto s3_id = ipm.add_subscription(s3);

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  auto original_message_pointer = unique_msg.get();
  auto p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer, unique_msg.get());
  }
  unique_msg.reset();

  ipm.take_intra_process_message(p1_id, p1_m1_id, s2_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer, unique_msg.get());
  }
  unique_msg.reset();

  ipm.take_intra_process_message(p1_id, p1_m1_id, s3_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    // Should match the original pointer.
    EXPECT_EQ(original_message_pointer, unique_msg.get());
  }
}

/*
   This tests normal operation with multiple publishers and one subscription.
   - Creates a publisher and three subscriptions on the same topic.
   - Publish a message.
   - Take with each subscription, checking that the last takes the original back.
 */
TEST(TestIntraProcessManager, multiple_publishers_one_subscription) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto p1 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p1->mock_topic_name = "nominal1";
  p1->mock_queue_size = 10;

  auto p2 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p2->mock_topic_name = "nominal1";
  p2->mock_queue_size = 10;

  auto p3 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p3->mock_topic_name = "nominal1";
  p3->mock_queue_size = 10;

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto p1_id = ipm.add_publisher(p1);
  auto p2_id = ipm.add_publisher(p2);
  auto p3_id = ipm.add_publisher(p3);
  auto s1_id = ipm.add_subscription(s1);

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  // First publish
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  auto original_message_pointer1 = unique_msg.get();
  auto p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  // Second publish
  ipm_msg->message_sequence = 43;
  ipm_msg->publisher_id = 43;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  auto original_message_pointer2 = unique_msg.get();
  auto p2_m1_id = ipm.store_intra_process_message(p2_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  // Third publish
  ipm_msg->message_sequence = 44;
  ipm_msg->publisher_id = 44;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  auto original_message_pointer3 = unique_msg.get();
  auto p3_m1_id = ipm.store_intra_process_message(p3_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  // First take
  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    EXPECT_EQ(original_message_pointer1, unique_msg.get());
  }
  unique_msg.reset();

  // Second take
  ipm.take_intra_process_message(p2_id, p2_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(43ul, unique_msg->message_sequence);
    EXPECT_EQ(43ul, unique_msg->publisher_id);
    EXPECT_EQ(original_message_pointer2, unique_msg.get());
  }
  unique_msg.reset();

  // Third take
  ipm.take_intra_process_message(p3_id, p3_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(44ul, unique_msg->message_sequence);
    EXPECT_EQ(44ul, unique_msg->publisher_id);
    EXPECT_EQ(original_message_pointer3, unique_msg.get());
  }
  unique_msg.reset();
}

/*
   This tests normal operation with multiple publishers and subscriptions.
   - Creates three publishers and three subscriptions on the same topic.
   - Publish a message on each publisher.
   - Take from each publisher with each subscription, checking the pointer.
 */
TEST(TestIntraProcessManager, multiple_publishers_multiple_subscription) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto p1 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p1->mock_topic_name = "nominal1";
  p1->mock_queue_size = 10;

  auto p2 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p2->mock_topic_name = "nominal1";
  p2->mock_queue_size = 10;

  auto p3 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p3->mock_topic_name = "nominal1";
  p3->mock_queue_size = 10;

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto s2 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s2->mock_topic_name = "nominal1";
  s2->mock_queue_size = 10;

  auto s3 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s3->mock_topic_name = "nominal1";
  s3->mock_queue_size = 10;

  auto p1_id = ipm.add_publisher(p1);
  auto p2_id = ipm.add_publisher(p2);
  auto p3_id = ipm.add_publisher(p3);
  auto s1_id = ipm.add_subscription(s1);
  auto s2_id = ipm.add_subscription(s2);
  auto s3_id = ipm.add_subscription(s3);

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  // First publish
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  auto original_message_pointer1 = unique_msg.get();
  auto p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  // Second publish
  ipm_msg->message_sequence = 43;
  ipm_msg->publisher_id = 43;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  auto original_message_pointer2 = unique_msg.get();
  auto p2_m1_id = ipm.store_intra_process_message(p2_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  // Third publish
  ipm_msg->message_sequence = 44;
  ipm_msg->publisher_id = 44;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  auto original_message_pointer3 = unique_msg.get();
  auto p3_m1_id = ipm.store_intra_process_message(p3_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  // First take
  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer1, unique_msg.get());
  }
  unique_msg.reset();

  ipm.take_intra_process_message(p1_id, p1_m1_id, s2_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer1, unique_msg.get());
  }
  unique_msg.reset();

  ipm.take_intra_process_message(p1_id, p1_m1_id, s3_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(42ul, unique_msg->message_sequence);
    EXPECT_EQ(42ul, unique_msg->publisher_id);
    EXPECT_EQ(original_message_pointer1, unique_msg.get());  // Final take.
  }
  unique_msg.reset();

  // Second take
  ipm.take_intra_process_message(p2_id, p2_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(43ul, unique_msg->message_sequence);
    EXPECT_EQ(43ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer2, unique_msg.get());
  }
  unique_msg.reset();

  ipm.take_intra_process_message(p2_id, p2_m1_id, s2_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(43ul, unique_msg->message_sequence);
    EXPECT_EQ(43ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer2, unique_msg.get());
  }
  unique_msg.reset();

  ipm.take_intra_process_message(p2_id, p2_m1_id, s3_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(43ul, unique_msg->message_sequence);
    EXPECT_EQ(43ul, unique_msg->publisher_id);
    EXPECT_EQ(original_message_pointer2, unique_msg.get());
  }
  unique_msg.reset();

  // Third take
  ipm.take_intra_process_message(p3_id, p3_m1_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(44ul, unique_msg->message_sequence);
    EXPECT_EQ(44ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer3, unique_msg.get());
  }
  unique_msg.reset();

  ipm.take_intra_process_message(p3_id, p3_m1_id, s2_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(44ul, unique_msg->message_sequence);
    EXPECT_EQ(44ul, unique_msg->publisher_id);
    EXPECT_NE(original_message_pointer3, unique_msg.get());
  }
  unique_msg.reset();

  ipm.take_intra_process_message(p3_id, p3_m1_id, s3_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(44ul, unique_msg->message_sequence);
    EXPECT_EQ(44ul, unique_msg->publisher_id);
    EXPECT_EQ(original_message_pointer3, unique_msg.get());
  }
  unique_msg.reset();
}

/*
   Tests displacing a message from the ring buffer before take is called.
   - Creates a publisher (buffer_size = 2) and a subscription on the same topic.
   - Publish a message on the publisher.
   - Publish another message.
   - Take the second message.
   - Publish a message.
   - Try to take the first message, should fail.
 */
TEST(TestIntraProcessManager, ring_buffer_displacement) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto p1 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p1->mock_topic_name = "nominal1";
  p1->mock_queue_size = 2;

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto p1_id = ipm.add_publisher(p1);
  auto s1_id = ipm.add_subscription(s1);

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  auto p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm_msg->message_sequence = 43;
  ipm_msg->publisher_id = 43;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  auto original_message_pointer2 = unique_msg.get();
  auto p1_m2_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  ipm.take_intra_process_message(p1_id, p1_m2_id, s1_id, unique_msg);
  EXPECT_NE(nullptr, unique_msg);
  if (unique_msg) {
    EXPECT_EQ(43ul, unique_msg->message_sequence);
    EXPECT_EQ(43ul, unique_msg->publisher_id);
    EXPECT_EQ(original_message_pointer2, unique_msg.get());
  }
  unique_msg.reset();

  ipm_msg->message_sequence = 44;
  ipm_msg->publisher_id = 44;
  unique_msg.reset(new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg));

  ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  EXPECT_EQ(nullptr, unique_msg);
  unique_msg.reset();

  // Since it just got displaced it should no longer be there to take.
  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_EQ(nullptr, unique_msg);
}

/*
   Simulates race condition where a subscription is created after publish.
   - Creates a publisher.
   - Publish a message on the publisher.
   - Create a subscription on the same topic.
   - Try to take the message with the newly created subscription, should fail.
 */
TEST(TestIntraProcessManager, subscription_creation_race_condition) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto p1 = std::make_shared<
    rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
    >();
  p1->mock_topic_name = "nominal1";
  p1->mock_queue_size = 2;

  auto p1_id = ipm.add_publisher(p1);

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  auto p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
  ASSERT_EQ(nullptr, unique_msg);

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto s1_id = ipm.add_subscription(s1);

  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_EQ(nullptr, unique_msg);
}

/*
   Simulates race condition where a publisher goes out of scope before take.
   - Create a subscription.
   - Creates a publisher on the same topic in a scope.
   - Publish a message on the publisher in a scope.
   - Let the scope expire.
   - Try to take the message with the subscription, should fail.
 */
TEST(TestIntraProcessManager, publisher_out_of_scope_take) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  auto s1 = std::make_shared<rclcpp::mock::SubscriptionBase>();
  s1->mock_topic_name = "nominal1";
  s1->mock_queue_size = 10;

  auto s1_id = ipm.add_subscription(s1);

  uint64_t p1_id;
  uint64_t p1_m1_id;
  {
    auto p1 = std::make_shared<
      rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
      >();
    p1->mock_topic_name = "nominal1";
    p1->mock_queue_size = 2;

    p1_id = ipm.add_publisher(p1);

    auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
    ipm_msg->message_sequence = 42;
    ipm_msg->publisher_id = 42;
    rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
      new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
    );

    p1_m1_id = ipm.store_intra_process_message(p1_id, std::move(unique_msg));
    ASSERT_EQ(nullptr, unique_msg);

    // Explicitly remove publisher from ipm (emulate's publisher's destructor).
    ipm.remove_publisher(p1_id);
  }

  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(nullptr);
  // Should fail because the publisher is out of scope.
  ipm.take_intra_process_message(p1_id, p1_m1_id, s1_id, unique_msg);
  EXPECT_EQ(nullptr, unique_msg);
}

/*
   Simulates race condition where a publisher goes out of scope before store.
   - Creates a publisher in a scope.
   - Let the scope expire.
   - Publish a message on the publisher in a scope, should throw.
 */
TEST(TestIntraProcessManager, publisher_out_of_scope_store) {
  rclcpp::intra_process_manager::IntraProcessManager ipm;

  uint64_t p1_id;
  {
    auto p1 = std::make_shared<
      rclcpp::mock::Publisher<rcl_interfaces::msg::IntraProcessMessage>
      >();
    p1->mock_topic_name = "nominal1";
    p1->mock_queue_size = 2;

    p1_id = ipm.add_publisher(p1);
  }

  auto ipm_msg = std::make_shared<rcl_interfaces::msg::IntraProcessMessage>();
  ipm_msg->message_sequence = 42;
  ipm_msg->publisher_id = 42;
  rcl_interfaces::msg::IntraProcessMessage::UniquePtr unique_msg(
    new rcl_interfaces::msg::IntraProcessMessage(*ipm_msg)
  );

  EXPECT_THROW(ipm.store_intra_process_message(p1_id, std::move(unique_msg)), std::runtime_error);
  ASSERT_EQ(nullptr, unique_msg);
}
