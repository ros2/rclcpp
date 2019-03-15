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

#ifndef RCLCPP__SUBSCRIPTION_OPTIONS_HPP_
#define RCLCPP__SUBSCRIPTION_OPTIONS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/visibility_control.hpp"
#include "rclcpp/qos_event.hpp"


namespace rclcpp
{

struct SubscriptionEventCallbacks
{
  QOSDeadlineEventCallbackType deadline_callback_;
  QOSLivelinessEventCallbackType liveliness_callback_;
  QOSLifespanEventCallbackType lifespan_callback_;
};

template<typename Alloc = std::allocator<void>>
class SubscriptionOptions
{
public:
  /// Create SubscriptionOptions with default values, optionally specifying the allocator to use.
  /**
   * Default values for the node options:
   *
   *   - deadline_callback = nullptr
   *   - liveliness_callback = nullptr
   *   - lifespan_callback = nullptr
   *   - subscription_qos_profile = rmw_qos_profile_default
   *   - ignore_local_publications = false
   *   - allocator = nullptr
   *
   * \param[in] allocator allocator to use in construction of SubscriptionOptions.
   */
  RCLCPP_PUBLIC
  SubscriptionOptions() = default;

  RCLCPP_PUBLIC
  explicit SubscriptionOptions(const rmw_qos_profile_t & qos_profile)
  : subscription_qos_profile_(qos_profile) {}

  /// Destructor.
  RCLCPP_PUBLIC
  virtual
  ~SubscriptionOptions() = default;

  /// Copy constructor.
  RCLCPP_PUBLIC
  SubscriptionOptions(const SubscriptionOptions & other) = default;

  /// Assignment operator.
  RCLCPP_PUBLIC
  SubscriptionOptions &
  operator=(const SubscriptionOptions & other) = default;


  /// Return a reference to the subscription_qos_profile QoS.
  RCLCPP_PUBLIC
  const rmw_qos_profile_t &
  qos_profile() const
  {
    return subscription_qos_profile_;
  }

  RCLCPP_PUBLIC
  rmw_qos_profile_t &
  qos_profile()
  {
    return subscription_qos_profile_;
  }

  /// Set the subscription_qos_profile QoS, return this for parameter idiom.
  /**
   * The QoS settings to be used for the subscription
   */
  RCLCPP_PUBLIC
  SubscriptionOptions &
  qos_profile(const rmw_qos_profile_t & subscription_qos_profile)
  {
    subscription_qos_profile_ = subscription_qos_profile;
    return *this;
  }

  RCLCPP_PUBLIC
  size_t &
  qos_history_depth()
  {
    return subscription_qos_profile_.depth;
  }

  RCLCPP_PUBLIC
  SubscriptionOptions &
  qos_history_depth(size_t depth)
  {
    subscription_qos_profile_.depth = depth;
    return *this;
  }


  RCLCPP_PUBLIC
  const QOSDeadlineEventCallbackType &
  deadline_callback() const
  {
    return callbacks_.deadline_callback_;
  }

  RCLCPP_PUBLIC
  SubscriptionOptions &
  deadline_callback(const QOSDeadlineEventCallbackType & callback)
  {
    callbacks_.deadline_callback_ = callback;
    return *this;
  }

  RCLCPP_PUBLIC
  const QOSLivelinessEventCallbackType &
  liveliness_callback() const
  {
    return callbacks_.liveliness_callback_;
  }

  RCLCPP_PUBLIC
  SubscriptionOptions &
  liveliness_callback(const QOSLivelinessEventCallbackType & callback)
  {
    callbacks_.liveliness_callback_ = callback;
    return *this;
  }

  RCLCPP_PUBLIC
  const QOSLifespanEventCallbackType &
  lifespan_callback() const
  {
    return callbacks_.lifespan_callback_;
  }

  RCLCPP_PUBLIC
  SubscriptionOptions &
  lifespan_callback(const QOSLifespanEventCallbackType & callback)
  {
    callbacks_.lifespan_callback_ = callback;
    return *this;
  }

  RCLCPP_PUBLIC
  const SubscriptionEventCallbacks &
  event_callbacks() const
  {
    return callbacks_;
  }


  RCLCPP_PUBLIC
  bool
  ignore_local_publications() const
  {
    return ignore_local_publications_;
  }

  RCLCPP_PUBLIC
  SubscriptionOptions &
  ignore_local_publications(bool ignore)
  {
    ignore_local_publications_ = ignore;
    return *this;
  }

  /// Return the std::shared_ptr<Alloc> to be used.
  RCLCPP_PUBLIC
  std::shared_ptr<Alloc>
  allocator() const
  {
    return allocator_;
  }

  RCLCPP_PUBLIC
  SubscriptionOptions &
  allocator(std::shared_ptr<Alloc> allocator)
  {
    allocator_ = allocator;
  }

private:
  // IMPORTANT: if any of these default values are changed, please update the
  // documentation in this class.

  SubscriptionEventCallbacks callbacks_;

  rmw_qos_profile_t subscription_qos_profile_ = rmw_qos_profile_default;

  bool ignore_local_publications_ = false;

  std::shared_ptr<Alloc> allocator_ = nullptr;
};

}  // namespace rclcpp

#endif  // RCLCPP__SUBSCRIPTION_OPTIONS_HPP_
