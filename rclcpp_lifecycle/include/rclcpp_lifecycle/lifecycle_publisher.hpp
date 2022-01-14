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

#include "rclcpp/logging.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/managed_entity.hpp"


namespace rclcpp_lifecycle
{
/// brief child class of rclcpp Publisher class.
/**
 * Overrides all publisher functions to check for enabled/disabled state.
 */
template<typename MessageT, typename Alloc = std::allocator<void>>
class LifecyclePublisher : public SimpleManagedEntity,
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
    should_log_(true),
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
    if (!this->is_activated()) {
      log_publisher_not_enabled();
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
    if (!this->is_activated()) {
      log_publisher_not_enabled();
      return;
    }
    rclcpp::Publisher<MessageT, Alloc>::publish(msg);
  }

  void
  on_activate() override
  {
    SimpleManagedEntity::on_activate();
    should_log_ = true;
  }

private:
  /// LifecyclePublisher log helper function
  /**
   * Helper function that logs a message saying that publisher can't publish
   * because it's not enabled.
   */
  void log_publisher_not_enabled()
  {
    // Nothing to do if we are not meant to log
    if (!should_log_) {
      return;
    }

    // Log the message
    RCLCPP_WARN(
      logger_,
      "Trying to publish message on the topic '%s', but the publisher is not activated",
      this->get_topic_name());

    // We stop logging until the flag gets enabled again
    should_log_ = false;
  }

  bool should_log_ = true;
  rclcpp::Logger logger_;
};

}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_PUBLISHER_HPP_
