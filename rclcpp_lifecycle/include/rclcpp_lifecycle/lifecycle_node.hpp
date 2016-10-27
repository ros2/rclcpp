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

#ifndef RCLCPP__LIFECYCLE_NODE_HPP_
#define RCLCPP__LIFECYCLE_NODE_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcl_lifecycle/lifecycle_state.h"

namespace rclcpp
{
namespace node
{

class LifecycleNode : public rclcpp::node::Node
{
public:

  RCLCPP_PUBLIC
  explicit LifecycleNode(const std::string & node_name, bool use_intra_process_comms = false) :
    Node(node_name, use_intra_process_comms),
    state_machine_(rcl_get_default_state_machine())
  {
    printf("Hello Lifecycle construcotr\n");
  }

  ~LifecycleNode(){}

  template<typename MessageT, typename Alloc = std::allocator<void>>
  std::shared_ptr<rclcpp::publisher::LifecyclePublisher<MessageT, Alloc>>
  create_publisher(
      const std::string & topic_name,
      const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
      std::shared_ptr<Alloc> allocator = nullptr)
  {
    auto pub =  rclcpp::node::Node::create_publisher<MessageT, Alloc,
      rclcpp::publisher::LifecyclePublisher<MessageT, Alloc>>(
        topic_name, qos_profile, allocator);

    weak_pub_ = pub;
    return pub;
  }

  void
  print_state_machine()
  {
    printf("Got primary states:\n");
    for (auto i=0; i<state_machine_.primary_states_size; ++i)
    {
      printf("\tState: %u\tLabel: %s\n",state_machine_.primary_states[i].state, state_machine_.primary_states[i].label);
    }
  }

  bool
  activate()
  {
    rcl_state_transition_t inactive_to_active = state_machine_.transitions[0];
    printf("First transition has state %u\n", inactive_to_active.start.state);
    // first transition in default state machine is inactive -> active
    if (!rcl_is_valid_transition(&state_machine_.current_state, &state_machine_.transitions[0]))
    {
      // TODO: go to error state here
      fprintf(stderr, "Unable to change from  current state %s from transition start %s\n", state_machine_.current_state.label, state_machine_.transitions[0].start.state);
      return false;
    }
    if (weak_pub_.expired())
    {
      // TODO: go to error state here
      fprintf(stderr, "I have no publisher handle\n");
      return false;
    }

    // TODO: does a return value make sense here?
    weak_pub_.lock()->on_activate();

    state_machine_.current_state.state = 1; // activated
    state_machine_.current_state.label = "active"; // activated
    return true;
  }

  bool
  deactivate()
  {
    // second transition is from active to deactive
    if(!rcl_is_valid_transition(&state_machine_.current_state, &state_machine_.transitions[1]))
    {
      fprintf(stderr, "Unable to change from  current state %s from transition start %s\n", state_machine_.current_state.label, state_machine_.transitions[1].start.state);
      fprintf(stderr, "deactivate is not a valid transaction in current state %s\n", state_machine_.current_state.label);
      // TODO: go to error state here
      return false;
    }
    if (weak_pub_.expired())
    {
      fprintf(stderr, "I have no publisher handle\n");
      // TODO: go to error state here
      return false;
    }

    // TODO: does a return value make sense here?
    weak_pub_.lock()->on_deactivate();

    state_machine_.current_state.state = 0; // activated
    state_machine_.current_state.label = "deactive";
    return true;
  }

private:

  std::weak_ptr<rclcpp::publisher::lifecycle_interface::Publisher> weak_pub_;

  rcl_state_machine_t state_machine_;
};

} // namespace node
} // namespace rclcpp

#endif
