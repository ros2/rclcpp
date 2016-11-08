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

#include <rclcpp_lifecycle/visibility_control.h>

namespace rclcpp
{
namespace node
{

namespace lifecycle
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
class LifecycleNodeInterface
{
public:
  virtual bool on_configure()  { return true; };
  virtual bool on_cleanup()    { return true; };
  virtual bool on_shutdown()   { return true; };
  virtual bool on_activate()   { return true; };
  virtual bool on_deactivate() { return true; };
  virtual bool on_error()      { return true; };
  // hardcoded mock
  // as we don't have a node base class yet
  virtual std::string get_name()
  {
    static auto counter = 0;
    std::string tmp_name = "my_node"+std::to_string(++counter);
    return tmp_name;
  }
};

/**
 * @brief LifecycleNode as child class of rclcpp Node
 * has lifecycle nodeinterface for configuring this node.
 */
class LifecycleNode : public rclcpp::node::Node, public lifecycle::LifecycleNodeInterface
{
public:

  LIFECYCLE_EXPORT
  explicit LifecycleNode(const std::string & node_name, bool use_intra_process_comms = false) :
    Node(node_name, use_intra_process_comms)
  {
  }

  LIFECYCLE_EXPORT
  ~LifecycleNode(){}

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

}  // namespace lifecycle_interface
}  // namespace node
}  // namespace rclcpp

#endif
