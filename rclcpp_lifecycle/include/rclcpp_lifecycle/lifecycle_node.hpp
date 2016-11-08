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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp
{
namespace node
{

namespace lifecycle
{
template<typename T, size_t State_Index, size_t Transition_Index>
struct Callback;

template<typename Ret, typename ... Params, size_t State_Index, size_t Transition_Index>
struct Callback<Ret(Params ...), State_Index, Transition_Index>
{
  template<typename ... Args>
  static Ret callback(Args ... args) {return func(args ...); }
  static std::function<Ret(Params ...)> func;
};

// Initialize the static member.
template<typename Ret, typename ... Params, size_t State_Index, size_t Transition_Index>
std::function<Ret(Params ...)> Callback<Ret(Params ...), State_Index, Transition_Index>::func;

/**
 * @brief Interface class for a managed node.
 * Pure virtual functions as defined in
 * http://design.ros2.org/articles/node_lifecycle.html
 */
class LifecycleNodeInterface
{
public:
  virtual bool on_configure() {return true; }
  virtual bool on_cleanup() {return true; }
  virtual bool on_shutdown() {return true; }
  virtual bool on_activate() {return true; }
  virtual bool on_deactivate() {return true; }
  virtual bool on_error() {return true; }
};

/**
 * @brief LifecycleNode as child class of rclcpp Node
 * has lifecycle nodeinterface for configuring this node.
 */
class LifecycleNode : public lifecycle::LifecycleNodeInterface
{
public:
  using LifecyclePublisherWeakPtr =
      std::weak_ptr<rclcpp::publisher::lifecycle_interface::PublisherInterface>;

  LIFECYCLE_EXPORT
  explicit LifecycleNode(const std::string & node_name, bool use_intra_process_comms = false)
  : base_interface_(std::make_shared<rclcpp::node::Node>(node_name, use_intra_process_comms)),
    communication_interface_(base_interface_)  // MOCK as base/comms interface not done yet
  {}

  LIFECYCLE_EXPORT
  ~LifecycleNode() {}

  // MOCK typedefs as node refactor not done yet
  using BaseInterface = rclcpp::node::Node;
  std::shared_ptr<BaseInterface>
  get_base_interface()
  {
    return base_interface_;
  }

  // MOCK typedefs as node refactor not done yet
  using CommunicationInterface = rclcpp::node::Node;
  std::shared_ptr<CommunicationInterface>
  get_communication_interface()
  {
    return communication_interface_;
  }

  std::string
  get_name()
  {
    return base_interface_->get_name();
  }

  /**
   * @brief same API for creating publisher as regular Node
   */
  template<typename MessageT, typename Alloc = std::allocator<void>>
  std::shared_ptr<rclcpp::publisher::LifecyclePublisher<MessageT, Alloc>>
  create_publisher(
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
    std::shared_ptr<Alloc> allocator = nullptr)
  {
    // create regular publisher in rclcpp::Node
    auto pub = communication_interface_->create_publisher<MessageT, Alloc,
      rclcpp::publisher::LifecyclePublisher<MessageT, Alloc>>(
      topic_name, qos_profile, allocator);

    // keep weak handle for this publisher to enable/disable afterwards
    weak_pubs_.push_back(pub);
    return pub;
  }

  template<typename CallbackType>
  typename rclcpp::timer::WallTimer<CallbackType>::SharedPtr
  create_wall_timer(
    std::chrono::nanoseconds period,
    CallbackType callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
  {
    return communication_interface_->create_wall_timer(period, callback, group);
  }

  LIFECYCLE_EXPORT
  virtual bool
  disable_communication()
  {
    for (auto weak_pub : weak_pubs_) {
      auto pub = weak_pub.lock();
      if (!pub) {
        return false;
      }
      pub->on_deactivate();
    }
    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  enable_communication()
  {
    for (auto weak_pub : weak_pubs_) {
      auto pub = weak_pub.lock();
      if (!pub) {
        return false;
      }
      pub->on_activate();
    }
    return true;
  }

private:
  std::shared_ptr<BaseInterface> base_interface_;
  std::shared_ptr<CommunicationInterface> communication_interface_;
  // Placeholder for all pub/sub/srv/clients
  std::vector<LifecyclePublisherWeakPtr> weak_pubs_;
};

}  // namespace lifecycle
}  // namespace node
}  // namespace rclcpp

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_
