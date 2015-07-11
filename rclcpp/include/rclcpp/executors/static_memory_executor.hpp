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

#ifndef RCLCPP_RCLCPP_EXECUTORS_STATIC_MEMORY_EXECUTOR_HPP_
#define RCLCPP_RCLCPP_EXECUTORS_STATIC_MEMORY_EXECUTOR_HPP_

#include <cassert>
#include <cstdlib>

#include <rmw/rmw.h>
#include <memory>
#include <vector>

#include <rclcpp/executor.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/rate.hpp>

#define MAX_SUBSCRIPTIONS 16
#define MAX_SERVICES 8
#define MAX_CLIENTS 16
#define MAX_GUARD_CONDS 100

namespace rclcpp {
namespace executors {
namespace static_memory_executor {

/* StaticMemoryExecutor provides separate statically allocated arrays
   for subscriptions, services, clients, and guard conditions and reimplements
   get_allocated_handles and remove_allocated_handles.
   The size of these arrays is defined at compile time. */
class StaticMemoryExecutor : public executor::Executor {
 public:
  RCLCPP_MAKE_SHARED_DEFINITIONS(StaticMemoryExecutor);

  StaticMemoryExecutor() {
    memset(subscriptions, 0, MAX_SUBSCRIPTIONS);
    memset(services, 0, MAX_SERVICES);
    memset(clients, 0, MAX_CLIENTS);
    memset(guard_conds, 0, MAX_GUARD_CONDS);
  }

  ~StaticMemoryExecutor() {}

  virtual void spin() {
    while (rclcpp::utilities::ok()) {
      auto any_exec = get_next_executable();
      execute_any_executable(any_exec);
    }
  }

 protected:
  void **get_allocated_handles(executor::executor_handle_t handle_type,
                               size_t size) {
    switch (handle_type) {
      case executor::subscriber_handle:
        if (size > MAX_SUBSCRIPTIONS) {
          std::cout << "subscriber error" << std::endl;
          throw std::runtime_error("Size of requested handle pointer exceeded" +
                                   " allocated memory for subscribers.");
        }
        return this->subscriptions;
      case executor::service_handle:
        if (size > MAX_SERVICES) {
          // TODO(jacquelinekay) change error if default impl is changed
          throw std::runtime_error("Size of requested handle pointer exceeded" +
                                   " allocated memory for services.");
        }
        return this->services;
      case executor::client_handle:
        if (size > MAX_CLIENTS) {
          // TODO(jacquelinekay) change error if default impl is changed
          throw std::runtime_error("Size of requested handle pointer exceeded" +
                                   " allocated memory for clients.");
        }
        return this->clients;
      case executor::guard_cond_handle:
        if (size > MAX_GUARD_CONDS) {
          // TODO(jacquelinekay) change error if default impl is changed
          throw std::runtime_error("Size of requested handle pointer exceeded" +
                                   " allocated memory for guard conditions.");
        }
        return this->guard_conds;
      default:
        break;
    }
    throw std::runtime_error
        ("Invalid enum type in StaticMemoryExecutor::get_allocated_handles");
  }

  void remove_allocated_handles(void **handle, size_t size) {
    if (handle == NULL) {
      std::cout << "warning: Null pointer passed to remove_allocated_handles."
                << std::endl;
      return;
    }

    memset(handle, 0, size);
  }

 private:
  RCLCPP_DISABLE_COPY(StaticMemoryExecutor);
  void *subscriptions[MAX_SUBSCRIPTIONS];
  void *services[MAX_SERVICES];
  void *clients[MAX_CLIENTS];
  void *guard_conds[MAX_GUARD_CONDS];
};

} /* namespace static_memory_executor */
} /* namespace executors */
} /* namespace rclcpp */

#endif /* RCLCPP_RCLCPP_EXECUTORS_STATIC_MEMORY_EXECUTOR_HPP_ */
