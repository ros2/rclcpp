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

#ifndef RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
#define RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace rclcpp
{
namespace lifecycle
{

// *INDENT-OFF*
enum class LifecyclePrimaryStatesT : unsigned int
{
  UNKNOWN       = 0,
  UNCONFIGURED  = 1,
  INACTIVE      = 2,
  ACTIVE        = 3,
  FINALIZED     = 4
};

enum class LifecycleTransitionsT : unsigned int
{
  CONFIGURING     = 10,
  CLEANINGUP      = 11,
  SHUTTINGDOWN    = 12,
  ACTIVATING      = 13,
  DEACTIVATING    = 14,
  ERRORPROCESSING = 15
};

class LifecycleManagerInterface
{
  virtual bool configure(const std::string& node_name = "")   = 0;
  virtual bool cleanup(const std::string& node_name = "")     = 0;
  virtual bool shutdown(const std::string& node_name = "")    = 0;
  virtual bool activate(const std::string& node_name = "")    = 0;
  virtual bool deactivate(const std::string& node_name = "")  = 0;
};
// *INDENT-ON*

class LifecycleManager : public LifecycleManagerInterface
{
public:
  using NodeInterfacePtr = std::shared_ptr<node::lifecycle::LifecycleNodeInterface>;
  using NodePtr = std::shared_ptr<node::lifecycle::LifecycleNode>;

  LIFECYCLE_EXPORT
  LifecycleManager();

  LIFECYCLE_EXPORT
  ~LifecycleManager();

  LIFECYCLE_EXPORT
  std::shared_ptr<rclcpp::node::Node>
  get_node_base_interface();

  LIFECYCLE_EXPORT
  void
  add_node_interface(const NodePtr & node);

  LIFECYCLE_EXPORT
  void
  add_node_interface(const std::string & node_name, const NodeInterfacePtr & node_interface);

  template<typename T>
  bool
  register_on_configure(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_configure(node_name, cb);
  }

  LIFECYCLE_EXPORT
  bool
  register_on_configure(const std::string & node_name, std::function<bool(void)> & fcn);

  LIFECYCLE_EXPORT
  bool
  configure(const std::string & node_name = "");

  template<typename T>
  bool
  register_on_cleanup(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_cleanup(node_name, cb);
  }

  LIFECYCLE_EXPORT
  bool
  register_on_cleanup(const std::string & node_name, std::function<bool(void)> & fcn);

  LIFECYCLE_EXPORT
  bool
  cleanup(const std::string & node_name = "");

  template<typename T>
  bool
  register_on_shutdown(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_shutdown(node_name, cb);
  }

  LIFECYCLE_EXPORT
  bool
  register_on_shutdown(const std::string & node_name, std::function<bool(void)> & fcn);

  LIFECYCLE_EXPORT
  bool
  shutdown(const std::string & node_name = "");

  template<typename T>
  bool
  register_on_activate(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_activate(node_name, cb);
  }

  LIFECYCLE_EXPORT
  bool
  register_on_activate(const std::string & node_name, std::function<bool(void)> & fcn);

  LIFECYCLE_EXPORT
  bool
  activate(const std::string & node_name = "");

  template<typename T>
  bool
  register_on_deactivate(const std::string & node_name, bool (T::* method)(void), T * instance)
  {
    auto cb = std::bind(method, instance);
    return register_on_deactivate(node_name, cb);
  }

  LIFECYCLE_EXPORT
  bool
  register_on_deactivate(const std::string & node_name, std::function<bool(void)> & fcn);

  LIFECYCLE_EXPORT
  bool
  deactivate(const std::string & node_name = "");

private:
  class LifecycleManagerImpl;
  std::unique_ptr<LifecycleManagerImpl> impl_;
};

}  // namespace lifecycle
}  // namespace rclcpp
#endif  // RCLCPP_LIFECYCLE__LIFECYCLE_MANAGER_HPP_
