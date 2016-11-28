// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CONTEXT_HPP_
#define RCLCPP__CONTEXT_HPP_

#include <iostream>
#include <memory>
#include <mutex>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/rmw.h"

namespace rclcpp
{

namespace context
{

class Context
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Context)

  RCLCPP_PUBLIC
  Context();

  template<typename SubContext, typename ... Args>
  std::shared_ptr<SubContext>
  get_sub_context(Args && ... args)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::type_index type_i(typeid(SubContext));
    std::shared_ptr<SubContext> sub_context;
    auto it = sub_contexts_.find(type_i);
    if (it == sub_contexts_.end()) {
      // It doesn't exist yet, make it
      // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
      sub_context = std::shared_ptr<SubContext>(
        new SubContext(std::forward<Args>(args) ...),
        [] (SubContext * sub_context_ptr) {
          delete sub_context_ptr;
        });
      // *INDENT-ON*
      sub_contexts_[type_i] = sub_context;
    } else {
      // It exists, get it out and cast it.
      sub_context = std::static_pointer_cast<SubContext>(it->second);
    }
    return sub_context;
  }

private:
  RCLCPP_DISABLE_COPY(Context)

  std::unordered_map<std::type_index, std::shared_ptr<void>> sub_contexts_;
  std::mutex mutex_;
};

}  // namespace context
}  // namespace rclcpp

#endif  // RCLCPP__CONTEXT_HPP_
