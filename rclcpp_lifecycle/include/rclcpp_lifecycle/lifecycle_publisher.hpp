// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__LIFECYCLE_PUBLISHER_HPP_
#define RCLCPP__LIFECYCLE_PUBLISHER_HPP_

#include "rclcpp/publisher.hpp"

namespace rclcpp
{
namespace publisher
{

namespace lifecycle_interface
{
/**
 * @brief base class with only
 * pure virtual functions. A managed
 * node can then deactivate or activate
 * the publishing.
 * It is more a convenient interface class
 * than a necessary base class.
 */
class PublisherInterface
{
public:
  virtual void on_activate() = 0;
  virtual void on_deactivate() = 0;
};
}  // namespace lifecycle_interface

/**
 * @brief child class of rclcpp Publisher class.
 * Overrides all publisher functions to check for
 * enabled/disabled state.
 */
template<typename MessageT, typename Alloc = std::allocator<void>>
class LifecyclePublisher : public rclcpp::publisher::Publisher<MessageT, Alloc>,
  public lifecycle_interface::PublisherInterface
{
public:
  using MessageAllocTraits = allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  LifecyclePublisher(
      std::shared_ptr<rcl_node_t> node_handle,
      std::string topic,
      const rcl_publisher_options_t & publisher_options,
      std::shared_ptr<MessageAlloc> allocator):
    ::rclcpp::publisher::Publisher<MessageT, Alloc>(node_handle, topic, publisher_options, allocator),
      enabled_(false)
  {  }

  ~LifecyclePublisher() {};

  /**
   * @brief briefly checks whether this publisher
   * was enabled or disabled and forwards the message
   * to the actual rclcp Publisher base class
   */
  virtual void
  publish(std::unique_ptr<MessageT, MessageDeleter> & msg)
  {
    if (!enabled_)
    {
      printf("I push every message to /dev/null\n");
      return;
    }
    rclcpp::publisher::Publisher<MessageT, Alloc>::publish(msg);
  }

  /**
   * @brief briefly checks whether this publisher
   * was enabled or disabled and forwards the message
   * to the actual rclcp Publisher base class
   */
  virtual void
  publish(const std::shared_ptr<MessageT> & msg)
  {
    if (!enabled_)
    {
      printf("I publish message %s to /dev/null\n", msg->data.c_str());
      return;
    }
    printf("I publish message %s to DDS\n", msg->data.c_str());
    rclcpp::publisher::Publisher<MessageT, Alloc>::publish(msg);
  }

  /**
   * @brief briefly checks whether this publisher
   * was enabled or disabled and forwards the message
   * to the actual rclcp Publisher base class
   */
  virtual void
  publish(std::shared_ptr<const MessageT> msg)
  {
    if (!enabled_)
    {
      printf("I am every message to /dev/null\n");
      return;
    }
    rclcpp::publisher::Publisher<MessageT, Alloc>::publish(msg);
  }

  /**
   * @brief briefly checks whether this publisher
   * was enabled or disabled and forwards the message
   * to the actual rclcp Publisher base class
   */
  virtual void
  publish(const MessageT & msg)
  {
    if (!enabled_)
    {
      printf("I am every message to /dev/null\n");
      return;
    }
    rclcpp::publisher::Publisher<MessageT, Alloc>::publish(msg);
  }

  /**
   * @brief briefly checks whether this publisher
   * was enabled or disabled and forwards the message
   * to the actual rclcp Publisher base class
   */
  virtual void
  publish(std::shared_ptr<const MessageT>& msg)
  {
    if (!enabled_)
    {
      printf("I am every message to /dev/null\n");
      return;
    }
    rclcpp::publisher::Publisher<MessageT, Alloc>::publish(msg);
  }

  virtual void
  on_activate()
  {
    printf("Lifecycle publisher is enabled\n");
    enabled_ = true;
  }

  virtual void
  on_deactivate()
  {
    printf("Lifecycle publisher is deactivated\n");
    enabled_ = false;
  }

private:
  bool enabled_ = false;
};

}  // namespace publisher
}  // namespace rclcpp

#endif  // endif RCLCPP__LIFECYCLE_PUBLISHER_HPP_
