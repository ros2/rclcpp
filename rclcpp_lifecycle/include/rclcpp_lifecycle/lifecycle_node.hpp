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
template <typename T, size_t State_Index, size_t Transition_Index>
struct Callback;

template <typename Ret, typename... Params, size_t State_Index, size_t Transition_Index>
struct Callback<Ret(Params...), State_Index, Transition_Index>
{
  template <typename... Args>
  static Ret callback(Args... args) { return func(args...); }
  static std::function<Ret(Params...)> func;
};

// Initialize the static member.
template <typename Ret, typename... Params, size_t State_Index, size_t Transition_Index>
std::function<Ret(Params...)> Callback<Ret(Params...), State_Index, Transition_Index>::func;

/**
 * @brief Interface class for a managed node.
 * Pure virtual functions as defined in
 * http://design.ros2.org/articles/node_lifecycle.html
 */
class NodeInterface
{
  std::map<size_t, std::function<bool(void)>> callback_map;
  rcl_state_machine_t state_machine_;

  protected:
  virtual bool on_configure()  { return true; };
  virtual bool on_cleanup()    { return true; };
  virtual bool on_shutdown()   { return true; };
  virtual bool on_activate()   { return true; };
  virtual bool on_deactivate() { return true; };
  virtual bool on_error()      { return true; };

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

  // In case with want to register the callbacks directly in c
  /*
  LIFECYCLE_EXPORT
  template<typename T, size_t State_Index, size_t Transition_Index>
  void
  register_state_callback(bool (T::*method)(), T* instance)
  {
    Callback<bool(), State_Index, Transition_Index>::func = std::bind(method, instance);
    bool(*c_function_pointer)(void) = static_cast<decltype(c_function_pointer)>(Callback<bool(), State_Index, Transition_Index>::callback);
    rcl_register_callback(&state_machine_, (unsigned int)State_Index, (unsigned int)Transition_Index, c_function_pointer);
  }
  */

  LIFECYCLE_EXPORT
  template<typename T>
  void
  register_transition_callback(bool (T::*method)(), T* instance, size_t transition_state_index)
  {
    const rcl_state_transition_t* transition_state
      = rcl_get_registered_transition_by_index(&state_machine_, transition_state_index);
    if (!transition_state)
    {
      // TODO do something smarter here
      throw std::runtime_error("Transition is not valid");
    }
    callback_map[transition_state_index] = std::bind(method, instance);
  }

  LIFECYCLE_EXPORT
  template<typename T>
  void
  register_transition_callback(bool (T::*method)(), T* instance, const std::string& transition_state_label)
  {
    const rcl_state_transition_t* transition_state
      = rcl_get_registered_transition_by_label(&state_machine_, transition_state_label.c_str());
    if (!transition_state)
    {
      // TODO do something smarter here
      throw std::runtime_error("Transition is not valid");
    }
    callback_map[transition_state->transition_state.index] = std::bind(method, instance);
  }

  public:
  LIFECYCLE_EXPORT
  void
  print_state_machine()
  {
    printf("current state is %s\n", state_machine_.current_state->label);
    rcl_print_transition_map(&state_machine_.transition_map);
  }

public:
  //virtual bool create()     = 0;
  virtual bool configure()
  {
    if (state_machine_.current_state->index == rcl_state_unconfigured.index)
    {
      // given the current state machine, specify a transition and go for it
      //auto ret = rcl_invoke_transition(&state_machine_, rcl_state_configuring);
      auto cb = callback_map[rcl_state_configuring.index];
      auto ret = cb();
      printf("%s\n", (ret)?"Callback was successful":"Callback unsuccessful");
      // change state here to "Configuring"
      //if (on_configure())
      //{
      //  state_machine_.current_state = &rcl_state_inactive;
      //  return true;
      //}
    }
    // everything else is considered wrong
    //state_machine_.current_state = rcl_state_error;
    return false;
  }

  virtual bool cleanup()
  {
    return on_cleanup();
  }

  virtual bool shutdown()
  {
    return on_shutdown();
  }

  virtual bool activate()
  {
    if (state_machine_.current_state->index == rcl_state_inactive.index)
    {
      // change state here to "Activating"
      if (on_activate())
      {
        state_machine_.current_state = &rcl_state_active;
        return true;
      }
    }
    // everything else is considered wrong
    //state_machine_.current_state = rcl_state_error;
    return false;
  }

  virtual bool deactivate()
  {
    if (state_machine_.current_state->index == rcl_state_active.index)
    {
      // change state here to "Activating"
      if (on_deactivate())
      {
        state_machine_.current_state = &rcl_state_inactive;
        return true;
      }
    }
    // everything else is considered wrong
    //state_machine_.current_state = rcl_state_error;
    return false;
  }
  //virtual bool destroy()    = 0;
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
    lifecycle_interface::NodeInterface::setup_state_machine();

    register_transition_callback(&LifecycleNode::on_configure, this, rcl_state_configuring.index);
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
  virtual bool
  on_configure()
  {
    // Placeholder print for all configuring work to be done
    // with each pub/sub/srv/client
    printf("I am doing some important configuration work\n");

    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_activate()
  {
    if (weak_pub_.expired())
    {
      // Someone externally destroyed the publisher handle
      fprintf(stderr, "I have no publisher handle\n");
      return false;
    }

    // Placeholder print for all configuring work to be done
    // with each pub/sub/srv/client
    printf("I am doing a lot of activation work\n");
    // TODO: does a return value make sense here?
    auto pub = weak_pub_.lock();
    if (!pub)
    {
      return false;
    }
    pub->on_activate();

    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_deactivate()
  {
    if (weak_pub_.expired())
    {
      fprintf(stderr, "I have no publisher handle\n");
      return false;
    }

    // Placeholder print for all configuring work to be done
    // with each pub/sub/srv/client
    printf("I am doing a lot of deactivation work\n");
    // TODO: does a return value make sense here?
    auto pub = weak_pub_.lock();
    if (!pub){
      return false;
    }
    pub->on_deactivate();

    return true;
  }

  LIFECYCLE_EXPORT
  virtual bool
  on_error()
  {
    fprintf(stderr, "Something went wrong here\n");
    return true;
  }

private:

  // weak handle for the managing publisher
  // TODO: Has to be a vector of weak publishers. Does on_deactivate deactivate every publisher?!
  // Placeholder for all pub/sub/srv/clients
  std::weak_ptr<rclcpp::publisher::lifecycle_interface::PublisherInterface> weak_pub_;

};

} // namespace node
} // namespace rclcpp

#endif
