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

#ifndef RCLCPP__LIFECYCLE_NODE_HPP_
#define RCLCPP__LIFECYCLE_NODE_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcl_lifecycle/lifecycle_state.h"
#include "rcl_lifecycle/default_state_machine.h"
#include "rcl_lifecycle/transition_map.h"

namespace rclcpp
{
namespace node
{

namespace lifecycle_interface
{

/**
 * @brief Interface class for a managed node.
 * Pure virtual functions as defined in
 * http://design.ros2.org/articles/node_lifecycle.html
 */
class NodeInterface
{
  virtual bool on_configure()   = 0;
  virtual bool on_clean_up()    = 0;
  virtual bool on_shutdown()    = 0;
  virtual bool on_activate()    = 0;
  virtual bool on_deactivate()  = 0;
  virtual bool on_error()       = 0;
};
}  // namespace lifecycle_interface

#include <rclcpp_lifecycle/visibility_control.h>

/**
 * @brief LifecycleNode as child class of rclcpp Node
 * has lifecycle nodeinterface for configuring this node.
 */
class LifecycleNode : public rclcpp::node::Node, public lifecycle_interface::NodeInterface
{
public:

  LIFECYCLE_EXPORT
  explicit LifecycleNode(const std::string & node_name, bool use_intra_process_comms = false) :
    Node(node_name, use_intra_process_comms)
  {
    setup_state_machine();
  }

  LIFECYCLE_EXPORT
  ~LifecycleNode(){}

  /**
   * @brief get the default state machine
   * as defined on design.ros.org
   */
  LIFECYCLE_EXPORT
  virtual void
  setup_state_machine()
  {
    state_machine_ = rcl_get_default_state_machine();
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
    auto pub =  rclcpp::node::Node::create_publisher<MessageT, Alloc,
      rclcpp::publisher::LifecyclePublisher<MessageT, Alloc>>(
        topic_name, qos_profile, allocator);

    // keep weak handle for this publisher to enable/disable afterwards
    weak_pub_ = pub;
    return pub;
  }

  LIFECYCLE_EXPORT
  void
  print_state_machine()
  {
    printf("current state is %s\n", state_machine_.current_state.label);
    rcl_print_transition_map(&state_machine_.transition_map);
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_configure()
  {
    // check on every function whether we are in the correct state
    /*if (!rcl_is_valid_transition(&state_machine_, &rcl_state_inactive))
    {
      // if not, go to error state
      state_machine_.current_state = rcl_state_error;
      return false;
    }*/

    // Placeholder print for all configuring work to be done
    // with each pub/sub/srv/client
    printf("I am doing some important configuration work\n");

    // work was done correctly, so change the current state
    state_machine_.current_state = rcl_state_inactive;
    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_clean_up()
  {
    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_shutdown()
  {
    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_activate()
  {
    /*if (!rcl_is_valid_transition(&state_machine_, &rcl_state_active))
    {
      fprintf(stderr, "Unable to change from  current state %s from transition start %s\n", state_machine_.current_state.label, rcl_state_active.label);
      state_machine_.current_state = rcl_state_error;
      return false;
    }*/
    if (weak_pub_.expired())
    {
      // Someone externally destroyed the publisher handle
      fprintf(stderr, "I have no publisher handle\n");
      state_machine_.current_state = rcl_state_error;
      return false;
    }

    // Placeholder print for all configuring work to be done
    // with each pub/sub/srv/client
    printf("I am doing a lot of activation work\n");
    // TODO: does a return value make sense here?
    weak_pub_.lock()->on_activate();

    state_machine_.current_state = rcl_state_active;
    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_deactivate()
  {
    // second transition is from active to deactive
    /*if(!rcl_is_valid_transition(&state_machine_, &rcl_state_inactive))
    {
      fprintf(stderr, "Unable to change from  current state %s from transition start %s\n", state_machine_.current_state.label, rcl_state_inactive.label);
      fprintf(stderr, "deactivate is not a valid transition in current state %s\n", state_machine_.current_state.label);
      // TODO: go to error state here
      return false;
    }*/
    if (weak_pub_.expired())
    {
      fprintf(stderr, "I have no publisher handle\n");
      // TODO: go to error state here
      return false;
    }

    // Placeholder print for all configuring work to be done
    // with each pub/sub/srv/client
    printf("I am doing a lot of activation work\n");
    // TODO: does a return value make sense here?
    weak_pub_.lock()->on_deactivate();

    state_machine_.current_state = rcl_state_inactive;
    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_error()
  {
    return true;
  }

private:

  // weak handle for the managing publisher
  // TODO: Has to be a vector of weak publishers. Does on_deactivate deactivate every publisher?!
  // Placeholder for all pub/sub/srv/clients
  std::weak_ptr<rclcpp::publisher::lifecycle_interface::PublisherInterface> weak_pub_;

  rcl_state_machine_t state_machine_;
};

} // namespace node
} // namespace rclcpp

#endif
