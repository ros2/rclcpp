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

#ifndef RCLCPP__PUBLISHER_OPTIONS_HPP_
#define RCLCPP__PUBLISHER_OPTIONS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/visibility_control.hpp"
#include "rclcpp/qos_event.hpp"


namespace rclcpp
{

struct PublisherEventCallbacks
{
  QOSDeadlineEventCallbackType deadline_callback_;
  QOSLivelinessEventCallbackType liveliness_callback_;
  QOSLifespanEventCallbackType lifespan_callback_;
};

template<typename Alloc = std::allocator<void>>
class PublisherOptions
{
public:
  /// Create PublisherOptions with default values, optionally specifying the allocator to use.
  /**
   * Default values for the node options:
   *
   *   - context = rclcpp::contexts::default_context::get_global_default_context()
   *   - arguments = {}
   *   - initial_parameters = {}
   *   - use_global_arguments = true
   *   - use_intra_process_comms = false
   *   - start_parameter_services = true
   *   - start_publisher_publisher = true
   *   - publisher_qos_profile = rmw_qos_profile_default
   *   - allocator = rcl_get_default_allocator()
   *
   * \param[in] allocator allocator to use in construction of PublisherOptions.
   */
  RCLCPP_PUBLIC
  PublisherOptions() = default;

  RCLCPP_PUBLIC
  explicit PublisherOptions(const rmw_qos_profile_t & qos_profile)
  : publisher_qos_profile_(qos_profile) {}

  /// Destructor.
  RCLCPP_PUBLIC
  virtual
  ~PublisherOptions() = default;

  /// Copy constructor.
  RCLCPP_PUBLIC
  PublisherOptions(const PublisherOptions & other) = default;

  /// Assignment operator.
  RCLCPP_PUBLIC
  PublisherOptions &
  operator=(const PublisherOptions & other) = default;


  /// Return a reference to the publisher_qos_profile QoS.
  RCLCPP_PUBLIC
  const rmw_qos_profile_t &
  qos_profile() const
  {
    return publisher_qos_profile_;
  }

  RCLCPP_PUBLIC
  rmw_qos_profile_t &
  qos_profile()
  {
    return publisher_qos_profile_;
  }

  /// Set the publisher_qos_profile QoS, return this for parameter idiom.
  /**
   * The QoS settings to be used for the publisher
   */
  RCLCPP_PUBLIC
  PublisherOptions &
  publisher_qos_profile(const rmw_qos_profile_t & publisher_qos_profile)
  {
    publisher_qos_profile_ = publisher_qos_profile;
    return *this;
  }

  RCLCPP_PUBLIC
  size_t &
  qos_history_depth()
  {
    return publisher_qos_profile_.depth;
  }

  RCLCPP_PUBLIC
  PublisherOptions &
  qos_history_depth(size_t depth)
  {
    publisher_qos_profile_.depth = depth;
    return *this;
  }


  RCLCPP_PUBLIC
  const QOSDeadlineEventCallbackType &
  deadline_callback() const
  {
    return callbacks_.deadline_callback_;
  }

  RCLCPP_PUBLIC
  PublisherOptions &
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
  PublisherOptions &
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
  PublisherOptions &
  lifespan_callback(const QOSLifespanEventCallbackType & callback)
  {
    callbacks_.lifespan_callback_ = callback;
    return *this;
  }

  RCLCPP_PUBLIC
  const PublisherEventCallbacks &
  event_callbacks() const
  {
    return callbacks_;
  }


  /// Return the rcl_allocator_t to be used.
  RCLCPP_PUBLIC
  std::shared_ptr<Alloc>
  allocator() const
  {
    return allocator_;
  }

  /// Set the rcl_allocator_t to be used, may cause deallocation of existing rcl_publisher_options_t
  /**
   * This will cause the internal rcl_publisher_options_t struct to be invalidated.
   */
  RCLCPP_PUBLIC
  PublisherOptions &
  allocator(std::shared_ptr<Alloc> allocator)
  {
    allocator_ = allocator;
  }

private:
  // IMPORTANT: if any of these default values are changed, please update the
  // documentation in this class.

  PublisherEventCallbacks callbacks_;

  rmw_qos_profile_t publisher_qos_profile_ = rmw_qos_profile_default;

  std::shared_ptr<Alloc> allocator_ = nullptr;
};

}  // namespace rclcpp

#endif  // RCLCPP__PUBLISHER_OPTIONS_HPP_
