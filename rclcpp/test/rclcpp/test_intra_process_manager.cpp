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

#include <gmock/gmock.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define RCLCPP_BUILDING_LIBRARY 1
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

// NOLINTNEXTLINE(build/include_order)
#include <rcl_interfaces/msg/log.hpp>

namespace rclcpp
{
// forward declaration
namespace experimental
{
class IntraProcessManager;
}  // namespace experimental

namespace mock
{

using IntraProcessManagerSharedPtr =
  std::shared_ptr<rclcpp::experimental::IntraProcessManager>;

using IntraProcessManagerWeakPtr =
  std::weak_ptr<rclcpp::experimental::IntraProcessManager>;

class PublisherBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PublisherBase)

  explicit PublisherBase(rclcpp::QoS qos = rclcpp::QoS(10))
  : qos_profile(qos),
    topic_name("topic")
  {}

  virtual ~PublisherBase()
  {}

  const char * get_topic_name() const
  {
    return topic_name.c_str();
  }

  void set_intra_process_manager(
    uint64_t intra_process_publisher_id,
    IntraProcessManagerSharedPtr ipm)
  {
    intra_process_publisher_id_ = intra_process_publisher_id;
    weak_ipm_ = ipm;
  }

  rclcpp::QoS
  get_actual_qos() const
  {
    return qos_profile;
  }

  bool
  operator==(const rmw_gid_t & gid) const
  {
    (void)gid;
    return false;
  }

  bool
  operator==(const rmw_gid_t * gid) const
  {
    (void)gid;
    return false;
  }

  rclcpp::QoS qos_profile;
  std::string topic_name;
  uint64_t intra_process_publisher_id_;
  IntraProcessManagerWeakPtr weak_ipm_;
};

template<typename T, typename Alloc = std::allocator<void>>
class Publisher : public PublisherBase
{
public:
  using MessageAllocTraits = allocator::AllocRebind<T, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, T>;
  using MessageUniquePtr = std::unique_ptr<T, MessageDeleter>;
  using MessageSharedPtr = std::shared_ptr<T>;

  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<T, Alloc>)

  explicit Publisher(rclcpp::QoS qos = rclcpp::QoS(10))
  : PublisherBase(qos)
  {
    auto allocator = std::make_shared<Alloc>();
    message_allocator_ = std::make_shared<MessageAlloc>(*allocator.get());
  }

  // The following functions use the IntraProcessManager
  // so they are declared after including it to avoid "invalid use of incomplete type"
  void publish(MessageUniquePtr msg);

  std::shared_ptr<MessageAlloc> message_allocator_;
};

}  // namespace mock
}  // namespace rclcpp

namespace rclcpp
{
namespace experimental
{
namespace buffers
{
namespace mock
{
template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename MessageDeleter = std::default_delete<MessageT>>
class IntraProcessBuffer
{
public:
  using ConstMessageSharedPtr = std::shared_ptr<const MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT>;

  RCLCPP_SMART_PTR_DEFINITIONS(IntraProcessBuffer)

  IntraProcessBuffer()
  {}

  void add(ConstMessageSharedPtr msg)
  {
    message_ptr = reinterpret_cast<std::uintptr_t>(msg.get());
    shared_msg = msg;
  }

  void add(MessageUniquePtr msg)
  {
    message_ptr = reinterpret_cast<std::uintptr_t>(msg.get());
    unique_msg = std::move(msg);
  }

  void pop(std::uintptr_t & msg_ptr)
  {
    msg_ptr = message_ptr;
    message_ptr = 0;
  }

  // need to store the messages somewhere otherwise the memory address will be reused
  ConstMessageSharedPtr shared_msg;
  MessageUniquePtr unique_msg;

  std::uintptr_t message_ptr;
};

}  // namespace mock
}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp


namespace rclcpp
{
namespace experimental
{
namespace mock
{

class SubscriptionIntraProcessBase
{
public:
  RCLCPP_SMART_PTR_ALIASES_ONLY(SubscriptionIntraProcessBase)

  explicit SubscriptionIntraProcessBase(
    rclcpp::Context::SharedPtr context,
    const std::string & topic = "topic",
    rclcpp::QoS qos = rclcpp::QoS(10))
  : qos_profile(qos), topic_name(topic)
  {
    (void)context;
  }

  virtual ~SubscriptionIntraProcessBase() {}

  virtual bool
  use_take_shared_method() const = 0;

  QoS
  get_actual_qos()
  {
    return qos_profile;
  }

  const char *
  get_topic_name()
  {
    return topic_name.c_str();
  }

  rclcpp::QoS qos_profile;
  std::string topic_name;
};

template<
  typename MessageT,
  typename Alloc = std::allocator<void>,
  typename Deleter = std::default_delete<MessageT>,
  typename ROSMessageType = MessageT
>
class SubscriptionIntraProcessBuffer : public SubscriptionIntraProcessBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionIntraProcessBuffer)

  explicit SubscriptionIntraProcessBuffer(rclcpp::QoS qos)
  : SubscriptionIntraProcessBase(nullptr, "topic", qos), take_shared_method(false)
  {
    buffer = std::make_unique<rclcpp::experimental::buffers::mock::IntraProcessBuffer<MessageT>>();
  }

  void
  provide_intra_process_message(std::shared_ptr<const MessageT> msg)
  {
    buffer->add(msg);
  }

  void
  provide_intra_process_message(std::unique_ptr<MessageT> msg)
  {
    buffer->add(std::move(msg));
  }

  void
  provide_intra_process_data(std::shared_ptr<const MessageT> msg)
  {
    buffer->add(msg);
  }

  void
  provide_intra_process_data(std::unique_ptr<MessageT> msg)
  {
    buffer->add(std::move(msg));
  }

  std::uintptr_t
  pop()
  {
    std::uintptr_t ptr;
    buffer->pop(ptr);
    return ptr;
  }

  bool
  use_take_shared_method() const
  {
    return take_shared_method;
  }

  bool take_shared_method;

  typename rclcpp::experimental::buffers::mock::IntraProcessBuffer<MessageT>::UniquePtr buffer;
};

template<
  typename MessageT,
  typename Alloc = std::allocator<MessageT>,
  typename Deleter = std::default_delete<MessageT>>
class SubscriptionIntraProcess : public SubscriptionIntraProcessBuffer<
    MessageT,
    Alloc,
    Deleter
>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriptionIntraProcess)

  explicit SubscriptionIntraProcess(rclcpp::QoS qos = rclcpp::QoS(10))
  : SubscriptionIntraProcessBuffer<MessageT, Alloc, Deleter>(qos)
  {
  }
};

}  // namespace mock
}  // namespace experimental
}  // namespace rclcpp

// Prevent the header files of the mocked classes to be included
#define RCLCPP__PUBLISHER_HPP_
#define RCLCPP__PUBLISHER_BASE_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BUFFER_HPP_
#define RCLCPP__EXPERIMENTAL__SUBSCRIPTION_INTRA_PROCESS_BASE_HPP_
// Force ipm to use our mock publisher class.
#define Publisher mock::Publisher
#define PublisherBase mock::PublisherBase
#define IntraProcessBuffer mock::IntraProcessBuffer
#define SubscriptionIntraProcessBase mock::SubscriptionIntraProcessBase
#define SubscriptionIntraProcessBuffer mock::SubscriptionIntraProcessBuffer
#define SubscriptionIntraProcess mock::SubscriptionIntraProcess
#include "../src/rclcpp/intra_process_manager.cpp"  // NOLINT
#undef Publisher
#undef PublisherBase
#undef IntraProcessBuffer
#undef SubscriptionIntraProcessBase
#undef SubscriptionIntraProcess

using ::testing::_;
using ::testing::UnorderedElementsAre;

namespace rclcpp
{
namespace mock
{

template<typename T, typename Alloc>
void Publisher<T, Alloc>::publish(MessageUniquePtr msg)
{
  auto ipm = weak_ipm_.lock();
  if (!ipm) {
    throw std::runtime_error(
            "intra process publish called after destruction of intra process manager");
  }
  if (!msg) {
    throw std::runtime_error("cannot publish msg which is a null pointer");
  }

  ipm->template do_intra_process_publish<T, T, Alloc>(
    intra_process_publisher_id_,
    std::move(msg),
    *message_allocator_);
}

}  // namespace mock
}  // namespace rclcpp

/*
   This tests how the class connects and disconnects publishers and subscriptions:
   - Creates 2 publishers on different topics and a subscription to one of them.
     Add everything to the intra-process manager.
   - All the entities are expected to have different ids.
   - Check the subscriptions count for each publisher.
   - One of the publishers is expected to have 1 subscription, while the other 0.
   - Check the subscription count for a non existing publisher id, should return 0.
   - Add a new publisher and a new subscription both with reliable QoS.
   - The subscriptions count of the previous publisher is expected to remain unchanged,
     while the new publisher is expected to have 2 subscriptions (it's compatible with both QoS).
   - Remove the just added subscriptions.
   - The count for the last publisher is expected to decrease to 1.
 */
TEST(TestIntraProcessManager, add_pub_sub) {
  using IntraProcessManagerT = rclcpp::experimental::IntraProcessManager;
  using MessageT = rcl_interfaces::msg::Log;
  using PublisherT = rclcpp::mock::Publisher<MessageT>;
  using SubscriptionIntraProcessT = rclcpp::experimental::mock::SubscriptionIntraProcess<MessageT>;

  auto ipm = std::make_shared<IntraProcessManagerT>();

  auto p1 = std::make_shared<PublisherT>(rclcpp::QoS(10).best_effort());

  auto p2 = std::make_shared<PublisherT>(rclcpp::QoS(10).best_effort());
  p2->topic_name = "different_topic_name";

  auto s1 = std::make_shared<SubscriptionIntraProcessT>(rclcpp::QoS(10).best_effort());

  auto p1_id = ipm->add_publisher(p1);
  auto p2_id = ipm->add_publisher(p2);
  auto s1_id = ipm->add_subscription(s1);

  bool unique_ids = p1_id != p2_id && p2_id != s1_id;
  ASSERT_TRUE(unique_ids);

  size_t p1_subs = ipm->get_subscription_count(p1_id);
  size_t p2_subs = ipm->get_subscription_count(p2_id);
  size_t non_existing_pub_subs = ipm->get_subscription_count(42);
  ASSERT_EQ(1u, p1_subs);
  ASSERT_EQ(0u, p2_subs);
  ASSERT_EQ(0u, non_existing_pub_subs);

  auto p3 = std::make_shared<PublisherT>(rclcpp::QoS(10).reliable());

  auto s2 = std::make_shared<SubscriptionIntraProcessT>(rclcpp::QoS(10).reliable());

  auto s2_id = ipm->add_subscription(s2);
  auto p3_id = ipm->add_publisher(p3);

  p1_subs = ipm->get_subscription_count(p1_id);
  p2_subs = ipm->get_subscription_count(p2_id);
  size_t p3_subs = ipm->get_subscription_count(p3_id);
  ASSERT_EQ(1u, p1_subs);
  ASSERT_EQ(0u, p2_subs);
  ASSERT_EQ(2u, p3_subs);

  ipm->remove_subscription(s2_id);
  p1_subs = ipm->get_subscription_count(p1_id);
  p2_subs = ipm->get_subscription_count(p2_id);
  p3_subs = ipm->get_subscription_count(p3_id);
  ASSERT_EQ(1u, p1_subs);
  ASSERT_EQ(0u, p2_subs);
  ASSERT_EQ(1u, p3_subs);
}

/*
   This tests the minimal usage of the class where there is a single subscription per publisher:
   - Publishes a unique_ptr message with a subscription requesting ownership.
   - The received message is expected to be the same.
   - Remove the first subscription from ipm and add a new one.
   - Publishes a unique_ptr message with a subscription not requesting ownership.
   - The received message is expected to be the same, the first subscription do not receive it.
   - Publishes a shared_ptr message with a subscription not requesting ownership.
   - The received message is expected to be the same.
 */
TEST(TestIntraProcessManager, single_subscription) {
  using IntraProcessManagerT = rclcpp::experimental::IntraProcessManager;
  using MessageT = rcl_interfaces::msg::Log;
  using PublisherT = rclcpp::mock::Publisher<MessageT>;
  using SubscriptionIntraProcessT = rclcpp::experimental::mock::SubscriptionIntraProcess<MessageT>;

  auto ipm = std::make_shared<IntraProcessManagerT>();

  auto p1 = std::make_shared<PublisherT>();
  auto p1_id = ipm->add_publisher(p1);
  p1->set_intra_process_manager(p1_id, ipm);

  auto s1 = std::make_shared<SubscriptionIntraProcessT>();
  s1->take_shared_method = false;
  auto s1_id = ipm->add_subscription(s1);

  auto unique_msg = std::make_unique<MessageT>();
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  auto received_message_pointer_1 = s1->pop();
  ASSERT_EQ(original_message_pointer, received_message_pointer_1);

  ipm->remove_subscription(s1_id);
  auto s2 = std::make_shared<SubscriptionIntraProcessT>();
  s2->take_shared_method = true;
  auto s2_id = ipm->add_subscription(s2);
  (void)s2_id;

  unique_msg = std::make_unique<MessageT>();
  original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  received_message_pointer_1 = s1->pop();
  auto received_message_pointer_2 = s2->pop();
  ASSERT_EQ(original_message_pointer, received_message_pointer_2);
  ASSERT_EQ(0u, received_message_pointer_1);

  unique_msg = std::make_unique<MessageT>();
  original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  received_message_pointer_2 = s2->pop();
  ASSERT_EQ(original_message_pointer, received_message_pointer_2);
}

/*
   This tests the usage of the class where there are multiple subscriptions of the same type:
   - Publishes a unique_ptr message with 2 subscriptions requesting ownership.
   - One is expected to receive the published message, while the other will receive a copy.
   - Publishes a unique_ptr message with 2 subscriptions not requesting ownership.
   - Both received messages are expected to be the same as the published one.
   - Publishes a shared_ptr message with 2 subscriptions requesting ownership.
   - Both received messages are expected to be a copy of the published one.
   - Publishes a shared_ptr message with 2 subscriptions not requesting ownership.
   - Both received messages are expected to be the same as the published one.
 */
TEST(TestIntraProcessManager, multiple_subscriptions_same_type) {
  using IntraProcessManagerT = rclcpp::experimental::IntraProcessManager;
  using MessageT = rcl_interfaces::msg::Log;
  using PublisherT = rclcpp::mock::Publisher<MessageT>;
  using SubscriptionIntraProcessT = rclcpp::experimental::mock::SubscriptionIntraProcess<MessageT>;

  auto ipm = std::make_shared<IntraProcessManagerT>();

  auto p1 = std::make_shared<PublisherT>();
  auto p1_id = ipm->add_publisher(p1);
  p1->set_intra_process_manager(p1_id, ipm);

  auto s1 = std::make_shared<SubscriptionIntraProcessT>();
  s1->take_shared_method = false;
  auto s1_id = ipm->add_subscription(s1);

  auto s2 = std::make_shared<SubscriptionIntraProcessT>();
  s2->take_shared_method = false;
  auto s2_id = ipm->add_subscription(s2);

  auto unique_msg = std::make_unique<MessageT>();
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  bool received_original_1 = s1->pop() == original_message_pointer;
  bool received_original_2 = s2->pop() == original_message_pointer;
  std::vector<bool> received_original_vec =
  {received_original_1, received_original_2};
  ASSERT_THAT(received_original_vec, UnorderedElementsAre(true, false));

  ipm->remove_subscription(s1_id);
  ipm->remove_subscription(s2_id);

  auto s3 = std::make_shared<SubscriptionIntraProcessT>();
  s3->take_shared_method = true;
  auto s3_id = ipm->add_subscription(s3);

  auto s4 = std::make_shared<SubscriptionIntraProcessT>();
  s4->take_shared_method = true;
  auto s4_id = ipm->add_subscription(s4);

  unique_msg = std::make_unique<MessageT>();
  original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  auto received_message_pointer_3 = s3->pop();
  auto received_message_pointer_4 = s4->pop();
  ASSERT_EQ(original_message_pointer, received_message_pointer_3);
  ASSERT_EQ(original_message_pointer, received_message_pointer_4);

  ipm->remove_subscription(s3_id);
  ipm->remove_subscription(s4_id);

  auto s5 = std::make_shared<SubscriptionIntraProcessT>();
  s5->take_shared_method = false;
  auto s5_id = ipm->add_subscription(s5);

  auto s6 = std::make_shared<SubscriptionIntraProcessT>();
  s6->take_shared_method = false;
  auto s6_id = ipm->add_subscription(s6);

  unique_msg = std::make_unique<MessageT>();
  original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  auto received_message_pointer_5 = s5->pop();
  auto received_message_pointer_6 = s6->pop();
  ASSERT_NE(original_message_pointer, received_message_pointer_5);
  // Someone gets the original unique_ptr, the last one to take.
  ASSERT_EQ(original_message_pointer, received_message_pointer_6);

  ipm->remove_subscription(s5_id);
  ipm->remove_subscription(s6_id);

  auto s7 = std::make_shared<SubscriptionIntraProcessT>();
  s7->take_shared_method = true;
  auto s7_id = ipm->add_subscription(s7);
  (void)s7_id;

  auto s8 = std::make_shared<SubscriptionIntraProcessT>();
  s8->take_shared_method = true;
  auto s8_id = ipm->add_subscription(s8);
  (void)s8_id;

  unique_msg = std::make_unique<MessageT>();
  original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  auto received_message_pointer_7 = s7->pop();
  auto received_message_pointer_8 = s8->pop();
  ASSERT_EQ(original_message_pointer, received_message_pointer_7);
  ASSERT_EQ(original_message_pointer, received_message_pointer_8);
}

/*
   This tests the usage of the class where there are multiple subscriptions of different types:
   - Publishes a unique_ptr message with 1 subscription requesting ownership and 1 not.
   - The one requesting ownership is expected to receive the published message,
     while the other is expected to receive a copy.
   - Publishes a unique_ptr message with 2 subscriptions requesting ownership and 1 not.
   - One of the subscriptions requesting ownership is expected to receive the published message,
     while both other subscriptions are expected to receive different copies.
   - Publishes a unique_ptr message with 2 subscriptions requesting ownership and 2 not.
   - The 2 subscriptions not requesting ownership are expected to both receive the same copy
     of the message, one of the subscription requesting ownership is expected to receive a
     different copy, while the last is expected to receive the published message.
   - Publishes a shared_ptr message with 1 subscription requesting ownership and 1 not.
   - The subscription requesting ownership is expected to receive a copy of the message, while
     the other is expected to receive the published message
 */
TEST(TestIntraProcessManager, multiple_subscriptions_different_type) {
  using IntraProcessManagerT = rclcpp::experimental::IntraProcessManager;
  using MessageT = rcl_interfaces::msg::Log;
  using PublisherT = rclcpp::mock::Publisher<MessageT>;
  using SubscriptionIntraProcessT = rclcpp::experimental::mock::SubscriptionIntraProcess<MessageT>;

  auto ipm = std::make_shared<IntraProcessManagerT>();

  auto p1 = std::make_shared<PublisherT>();
  auto p1_id = ipm->add_publisher(p1);
  p1->set_intra_process_manager(p1_id, ipm);

  auto s1 = std::make_shared<SubscriptionIntraProcessT>();
  s1->take_shared_method = true;
  auto s1_id = ipm->add_subscription(s1);

  auto s2 = std::make_shared<SubscriptionIntraProcessT>();
  s2->take_shared_method = false;
  auto s2_id = ipm->add_subscription(s2);

  auto unique_msg = std::make_unique<MessageT>();
  auto original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  auto received_message_pointer_1 = s1->pop();
  auto received_message_pointer_2 = s2->pop();
  ASSERT_NE(original_message_pointer, received_message_pointer_1);
  ASSERT_EQ(original_message_pointer, received_message_pointer_2);

  ipm->remove_subscription(s1_id);
  ipm->remove_subscription(s2_id);

  auto s3 = std::make_shared<SubscriptionIntraProcessT>();
  s3->take_shared_method = false;
  auto s3_id = ipm->add_subscription(s3);

  auto s4 = std::make_shared<SubscriptionIntraProcessT>();
  s4->take_shared_method = false;
  auto s4_id = ipm->add_subscription(s4);

  auto s5 = std::make_shared<SubscriptionIntraProcessT>();
  s5->take_shared_method = true;
  auto s5_id = ipm->add_subscription(s5);

  unique_msg = std::make_unique<MessageT>();
  original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  auto received_message_pointer_3 = s3->pop();
  auto received_message_pointer_4 = s4->pop();
  auto received_message_pointer_5 = s5->pop();
  bool received_original_3 = received_message_pointer_3 == original_message_pointer;
  bool received_original_4 = received_message_pointer_4 == original_message_pointer;
  bool received_original_5 = received_message_pointer_5 == original_message_pointer;
  std::vector<bool> received_original_vec =
  {received_original_3, received_original_4, received_original_5};
  ASSERT_THAT(received_original_vec, UnorderedElementsAre(true, false, false));
  ASSERT_NE(received_message_pointer_3, received_message_pointer_4);
  ASSERT_NE(received_message_pointer_5, received_message_pointer_3);
  ASSERT_NE(received_message_pointer_5, received_message_pointer_4);

  ipm->remove_subscription(s3_id);
  ipm->remove_subscription(s4_id);
  ipm->remove_subscription(s5_id);

  auto s6 = std::make_shared<SubscriptionIntraProcessT>();
  s6->take_shared_method = true;
  auto s6_id = ipm->add_subscription(s6);

  auto s7 = std::make_shared<SubscriptionIntraProcessT>();
  s7->take_shared_method = true;
  auto s7_id = ipm->add_subscription(s7);

  auto s8 = std::make_shared<SubscriptionIntraProcessT>();
  s8->take_shared_method = false;
  auto s8_id = ipm->add_subscription(s8);

  auto s9 = std::make_shared<SubscriptionIntraProcessT>();
  s9->take_shared_method = false;
  auto s9_id = ipm->add_subscription(s9);

  unique_msg = std::make_unique<MessageT>();
  original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  auto received_message_pointer_6 = s6->pop();
  auto received_message_pointer_7 = s7->pop();
  auto received_message_pointer_8 = s8->pop();
  auto received_message_pointer_9 = s9->pop();
  bool received_original_8 = received_message_pointer_8 == original_message_pointer;
  bool received_original_9 = received_message_pointer_9 == original_message_pointer;
  received_original_vec = {received_original_8, received_original_9};
  ASSERT_EQ(received_message_pointer_6, received_message_pointer_7);
  ASSERT_NE(original_message_pointer, received_message_pointer_6);
  ASSERT_NE(original_message_pointer, received_message_pointer_7);
  ASSERT_THAT(received_original_vec, UnorderedElementsAre(true, false));
  ASSERT_NE(received_message_pointer_8, received_message_pointer_6);
  ASSERT_NE(received_message_pointer_9, received_message_pointer_6);

  ipm->remove_subscription(s6_id);
  ipm->remove_subscription(s7_id);
  ipm->remove_subscription(s8_id);
  ipm->remove_subscription(s9_id);

  auto s10 = std::make_shared<SubscriptionIntraProcessT>();
  s10->take_shared_method = false;
  auto s10_id = ipm->add_subscription(s10);
  (void)s10_id;

  auto s11 = std::make_shared<SubscriptionIntraProcessT>();
  s11->take_shared_method = true;
  auto s11_id = ipm->add_subscription(s11);
  (void)s11_id;

  unique_msg = std::make_unique<MessageT>();
  original_message_pointer = reinterpret_cast<std::uintptr_t>(unique_msg.get());
  p1->publish(std::move(unique_msg));
  auto received_message_pointer_10 = s10->pop();
  auto received_message_pointer_11 = s11->pop();
  EXPECT_EQ(original_message_pointer, received_message_pointer_10);
  EXPECT_NE(original_message_pointer, received_message_pointer_11);
}
