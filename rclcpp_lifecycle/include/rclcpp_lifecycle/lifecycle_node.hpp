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
#include "rclcpp/timer.hpp"

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp
{
namespace lifecycle
{
/**
 * @brief Interface class for a managed node.
 * Pure virtual functions as defined in
 * http://design.ros2.org/articles/node_lifecycle.html
 */
// *INDENT-OFF
class LifecycleNodeInterface
{
public:
  virtual bool on_configure()   = 0;
  virtual bool on_cleanup()     = 0;
  virtual bool on_shutdown()    = 0;
  virtual bool on_activate()    = 0;
  virtual bool on_deactivate()  = 0;
  virtual bool on_error()       = 0;
};

class AbstractLifecycleNode : public LifecycleNodeInterface
{
public:
  virtual bool on_configure()   {return true;};
  virtual bool on_cleanup()     {return true;};
  virtual bool on_shutdown()    {return true;};
  virtual bool on_activate()    {return true;};
  virtual bool on_deactivate()  {return true;};
  virtual bool on_error()       {return true;};
};
// *INDENT-ON

/**
 * @brief LifecycleNode as child class of rclcpp Node
 * has lifecycle nodeinterface for configuring this node.
 */
class LifecycleNode : public AbstractLifecycleNode
{
public:
  LIFECYCLE_EXPORT
  explicit LifecycleNode(const std::string & node_name, bool use_intra_process_comms = false);

  LIFECYCLE_EXPORT
  virtual ~LifecycleNode();

  // MOCK typedefs as node refactor not done yet
  using BaseInterface = rclcpp::node::Node;
  std::shared_ptr<BaseInterface>
  get_base_interface()
  {
    return base_node_handle_;
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
    return base_node_handle_->get_name();
  }

  bool
  register_on_configure(std::function<bool(void)> fcn);
  bool
  register_on_cleanup(std::function<bool(void)> fcn);
  bool
  register_on_shutdown(std::function<bool(void)> fcn);
  bool
  register_on_activate(std::function<bool(void)> fcn);
  bool
  register_on_deactivate(std::function<bool(void)> fcn);
  bool
  register_on_error(std::function<bool(void)> fcn);

  /**
   * @brief same API for creating publisher as regular Node
   */
  template<typename MessageT, typename Alloc = std::allocator<void>>
  std::shared_ptr<rclcpp::lifecycle::LifecyclePublisher<MessageT, Alloc>>
  create_publisher(
    const std::string & topic_name,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
    std::shared_ptr<Alloc> allocator = nullptr)
  {
    // create regular publisher in rclcpp::Node
    auto pub = communication_interface_->create_publisher<MessageT, Alloc,
      rclcpp::lifecycle::LifecyclePublisher<MessageT, Alloc>>(
      topic_name, qos_profile, allocator);
    add_publisher_handle(pub);

    return pub;
  }


  template<typename CallbackType>
  typename rclcpp::timer::WallTimer<CallbackType>::SharedPtr
  create_wall_timer(
    std::chrono::nanoseconds period,
    CallbackType callback,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
  {
    auto timer = communication_interface_->create_wall_timer(period, callback, group);
    add_timer_handle(timer);

    return timer;
  }

protected:
  LIFECYCLE_EXPORT
  void
  add_publisher_handle(std::shared_ptr<rclcpp::lifecycle::LifecyclePublisherInterface> pub);

  LIFECYCLE_EXPORT
  void
  add_timer_handle(std::shared_ptr<rclcpp::timer::TimerBase> timer);

private:
  std::shared_ptr<BaseInterface> base_node_handle_;
  std::shared_ptr<CommunicationInterface> communication_interface_;
  class LifecycleNodeImpl;
  std::unique_ptr<LifecycleNodeImpl> impl_;
};

}  // namespace lifecycle
}  // namespace rclcpp

#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_NODE_HPP_
