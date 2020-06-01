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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_PUBLISHER_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/publisher.hpp"

#include "rclcpp/logging.hpp"

namespace rclcpp_lifecycle
{
/// base class with only
/**
 * pure virtual functions. A managed
 * node can then deactivate or activate
 * the publishing.
 * It is more a convenient interface class
 * than a necessary base class.
 */
class LifecyclePublisherInterface
{
public:
  virtual ~LifecyclePublisherInterface() {}
  virtual void on_activate() = 0;
  virtual void on_deactivate() = 0;
  virtual bool is_activated() = 0;
};

/// brief child class of rclcpp Publisher class.
/**
 * Overrides all publisher functions to check for enabled/disabled state.
 */
template<typename MessageT, typename Alloc = std::allocator<void>>
class LifecyclePublisher : public LifecyclePublisherInterface,
  public rclcpp::Publisher<MessageT, Alloc>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LifecyclePublisher)

  using MessageAllocTraits = rclcpp::allocator::AllocRebind<MessageT, Alloc>;
  using MessageAlloc = typename MessageAllocTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAlloc, MessageT>;
  using MessageUniquePtr = std::unique_ptr<MessageT, MessageDeleter>;

  LifecyclePublisher(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    const std::string & topic,
    const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<Alloc> & options)
  : rclcpp::Publisher<MessageT, Alloc>(node_base, topic, qos, options),
    enabled_(false),
    logger_(rclcpp::get_logger("LifecyclePublisher"))
  {
  }

  ~LifecyclePublisher() {}

  /// LifecyclePublisher publish function
  /**
   * The publish function checks whether the communication
   * was enabled or disabled and forwards the message
   * to the actual rclcpp Publisher base class
   */
  virtual void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    if (!enabled_) {
      RCLCPP_WARN(
        logger_,
        "Trying to publish message on the topic '%s', but the publisher is not activated",
        this->get_topic_name());

      return;
    }
    rclcpp::Publisher<MessageT, Alloc>::publish(std::move(msg));
  }

  /// LifecyclePublisher publish function
  /**
   * The publish function checks whether the communication
   * was enabled or disabled and forwards the message
   * to the actual rclcpp Publisher base class
   */
  virtual void
  publish(const MessageT & msg)
  {
    if (!enabled_) {
      RCLCPP_WARN(
        logger_,
        "Trying to publish message on the topic '%s', but the publisher is not activated",
        this->get_topic_name());

      return;
    }
    rclcpp::Publisher<MessageT, Alloc>::publish(msg);
  }

  virtual void
  on_activate()
  {
    enabled_ = true;
  }

  virtual void
  on_deactivate()
  {
    enabled_ = false;
  }

  virtual bool
  is_activated()
  {
    return enabled_;
  }

private:
  bool enabled_ = false;
  rclcpp::Logger logger_;
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_PUBLISHER_HPP_
