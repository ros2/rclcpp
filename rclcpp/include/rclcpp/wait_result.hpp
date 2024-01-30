// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_RESULT_HPP_
#define RCLCPP__WAIT_RESULT_HPP_

#include <cassert>
#include <functional>
#include <memory>
#include <stdexcept>

#include "rcl/wait.h"

#include "rclcpp/macros.hpp"
#include "rclcpp/wait_result_kind.hpp"

#include "rclcpp/timer.hpp"

namespace rclcpp
{

// TODO(wjwwood): the union-like design of this class could be replaced with
//   std::variant, when we have access to that...
/// Interface for introspecting a wait set after waiting on it.
/**
 * This class:
 *
 *   - provides the result of waiting, i.e. ready, timeout, or empty, and
 *   - holds the ownership of the entities of the wait set, if needed, and
 *   - provides the necessary information for iterating over the wait set.
 *
 * This class is only valid as long as the wait set which created it is valid,
 * and it must be deleted before the wait set is deleted, as it contains a
 * back reference to the wait set.
 *
 * An instance of this, which is returned from rclcpp::WaitSetTemplate::wait(),
 * will cause the wait set to keep ownership of the entities because it only
 * holds a reference to the sequences of them, rather than taking a copy.
 * Also, in the thread-safe case, an instance of this will cause the wait set,
 * to block calls which modify the sequences of the entities, e.g. add/remove
 * guard condition or subscription methods.
 *
 * \tparam WaitSetT The wait set type which created this class.
 */
template<class WaitSetT>
class WaitResult final
{
public:
  /// Create WaitResult from a "ready" result.
  /**
   * \param[in] wait_set A reference to the wait set, which this class
   *   will keep for the duration of its lifetime.
   * \return a WaitResult from a "ready" result.
   */
  static
  WaitResult
  from_ready_wait_result_kind(WaitSetT & wait_set)
  {
    return WaitResult(WaitResultKind::Ready, wait_set);
  }

  /// Create WaitResult from a "timeout" result.
  static
  WaitResult
  from_timeout_wait_result_kind()
  {
    return WaitResult(WaitResultKind::Timeout);
  }

  /// Create WaitResult from a "empty" result.
  static
  WaitResult
  from_empty_wait_result_kind()
  {
    return WaitResult(WaitResultKind::Empty);
  }

  /// Return the kind of the WaitResult.
  WaitResultKind
  kind() const
  {
    return wait_result_kind_;
  }

  /// Return the rcl wait set.
  /**
   * \return const rcl wait set.
   * \throws std::runtime_error if the class cannot access wait set when the result was not ready
   */
  const WaitSetT &
  get_wait_set() const
  {
    if (this->kind() != WaitResultKind::Ready) {
      throw std::runtime_error("cannot access wait set when the result was not ready");
    }
    // This should never happen, defensive (and debug mode) check only.
    assert(wait_set_pointer_);
    return *wait_set_pointer_;
  }

  /// Return the rcl wait set.
  /**
   * \return rcl wait set.
   * \throws std::runtime_error if the class cannot access wait set when the result was not ready
   */
  WaitSetT &
  get_wait_set()
  {
    if (this->kind() != WaitResultKind::Ready) {
      throw std::runtime_error("cannot access wait set when the result was not ready");
    }
    // This should never happen, defensive (and debug mode) check only.
    assert(wait_set_pointer_);
    return *wait_set_pointer_;
  }

  WaitResult(WaitResult && other) noexcept
  : wait_result_kind_(other.wait_result_kind_),
    wait_set_pointer_(std::exchange(other.wait_set_pointer_, nullptr))
  {}

  ~WaitResult()
  {
    if (wait_set_pointer_) {
      wait_set_pointer_->wait_result_release();
    }
  }

  size_t ready_timers() const
  {
    auto ready = 0;
    if (this->kind() == WaitResultKind::Ready) {
      const auto rcl_wait_set = wait_set_pointer_->get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_timers; ++ii) {
        if (rcl_wait_set.timers[ii] != nullptr &&
          wait_set_pointer_->timers(ii)->call())
        {
          ready++;
        }
      }
    }
    return ready;
  }

  size_t ready_subscriptions() const
  {
    auto ready = 0;
    if (this->kind() == WaitResultKind::Ready) {
      const auto rcl_wait_set = wait_set_pointer_->get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_subscriptions; ++ii) {
        if (rcl_wait_set.subscriptions[ii] != nullptr) {
          ready++;
        }
      }
    }
    return ready;
  }

  size_t ready_services() const
  {
    auto ready = 0;
    if (this->kind() == WaitResultKind::Ready) {
      const auto rcl_wait_set = wait_set_pointer_->get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_services; ++ii) {
        if (rcl_wait_set.services[ii] != nullptr) {
          ready++;
        }
      }
    }
    return ready;
  }

  size_t ready_clients() const
  {
    auto ready = 0;
    if (this->kind() == WaitResultKind::Ready) {
      const auto rcl_wait_set = wait_set_pointer_->get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_clients; ++ii) {
        if (rcl_wait_set.clients[ii] != nullptr) {
          ready++;
        }
      }
    }
    return ready;
  }

  size_t ready_waitables() const
  {
    auto ready = 0;
    if (this->kind() == WaitResultKind::Ready) {
      const auto rcl_wait_set = wait_set_pointer_->get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_waitables; ++ii) {
        if (rcl_wait_set.waitables[ii] != nullptr &&
          wait_set_pointer_->waitables[ii]->is_ready(&rcl_wait_set))
        {
          ready++;
        }
      }
    }
    return ready;
  }

  size_t ready_count() const
  {
    return ready_timers() +
           ready_subscriptions() +
           ready_services() +
           ready_clients() +
           ready_waitables();
  }

  std::shared_ptr<rclcpp::TimerBase> next_ready_timer()
  {
    auto ret = std::shared_ptr<rclcpp::TimerBase>{nullptr};
    if (this->kind() == WaitResultKind::Ready) {
      auto & rcl_wait_set = wait_set_pointer_->storage_get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_timers; ++ii) {
        if (rcl_wait_set.timers[ii] != nullptr &&
          wait_set_pointer_->timers(ii)->call())
        {
          ret = wait_set_pointer_->timers(ii);
          rcl_wait_set.timers[ii] = nullptr;
          break;
        }
      }
    }
    return ret;
  }

  std::shared_ptr<rclcpp::SubscriptionBase> next_ready_subscription()
  {
    auto ret = std::shared_ptr<rclcpp::SubscriptionBase>{nullptr};
    if (this->kind() == WaitResultKind::Ready) {
      auto & rcl_wait_set = wait_set_pointer_->storage_get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_subscriptions; ++ii) {
        if (rcl_wait_set.subscriptions[ii] != nullptr) {
          ret = wait_set_pointer_->subscriptions(ii);
          rcl_wait_set.subscriptions[ii] = nullptr;
          break;
        }
      }
    }
    return ret;
  }

  std::shared_ptr<rclcpp::ServiceBase> next_ready_service()
  {
    auto ret = std::shared_ptr<rclcpp::ServiceBase>{nullptr};
    if (this->kind() == WaitResultKind::Ready) {
      auto & rcl_wait_set = wait_set_pointer_->storage_get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_services; ++ii) {
        if (rcl_wait_set.services[ii] != nullptr) {
          ret = wait_set_pointer_->services(ii);
          rcl_wait_set.services[ii] = nullptr;
          break;
        }
      }
    }
    return ret;
  }

  std::shared_ptr<rclcpp::ClientBase> next_ready_client()
  {
    auto ret = std::shared_ptr<rclcpp::ClientBase>{nullptr};
    if (this->kind() == WaitResultKind::Ready) {
      auto & rcl_wait_set = wait_set_pointer_->storage_get_rcl_wait_set();
      for (size_t ii = 0; ii < rcl_wait_set.size_of_clients; ++ii) {
        if (rcl_wait_set.clients[ii] != nullptr) {
          ret = wait_set_pointer_->clients(ii);
          rcl_wait_set.clients[ii] = nullptr;
          break;
        }
      }
    }
    return ret;
  }

  std::shared_ptr<rclcpp::Waitable> next_ready_waitable()
  {
    auto ret = std::shared_ptr<rclcpp::Waitable>{nullptr};
    if (this->kind() == WaitResultKind::Ready) {
      auto rcl_wait_set = get_wait_set().get_rcl_wait_set();
      for (size_t ii = 0; ii < wait_set_pointer_->size_of_waitables(); ++ii) {
        if (wait_set_pointer_->waitables(ii)->is_ready(&rcl_wait_set)) {
          ret = wait_set_pointer_->waitables(ii);
          break;
        }
      }
    }
    return ret;
  }

private:
  RCLCPP_DISABLE_COPY(WaitResult)

  explicit WaitResult(WaitResultKind wait_result_kind)
  : wait_result_kind_(wait_result_kind)
  {
    // Should be enforced by the static factory methods on this class.
    assert(WaitResultKind::Ready != wait_result_kind);
  }

  WaitResult(WaitResultKind wait_result_kind, WaitSetT & wait_set)
  : wait_result_kind_(wait_result_kind),
    wait_set_pointer_(&wait_set)
  {
    // Should be enforced by the static factory methods on this class.
    assert(WaitResultKind::Ready == wait_result_kind);
    // Secure thread-safety (if provided) and shared ownership (if needed).
    wait_set_pointer_->wait_result_acquire();
  }

  const WaitResultKind wait_result_kind_;

  WaitSetT * wait_set_pointer_ = nullptr;
};

}  // namespace rclcpp

#endif  // RCLCPP__WAIT_RESULT_HPP_
