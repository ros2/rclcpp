#pragma once
#include <rclcpp/callback_group.hpp>
#include <rclcpp/guard_condition.hpp>
#include <vector>

namespace rclcpp::executors
{

/**
 * Snapshot of the state of a callback group.
 *
 * Contains all registered entities from the point in timer
 * when the snapshot was taken, as weak ptrs.
 */
struct CallbackGroupState
{
  CallbackGroupState(rclcpp::CallbackGroup & cbg)
  {
    update(cbg);
  }

  /// Update the state from the callback group
  void update(rclcpp::CallbackGroup & cbg)
  {
    cbg.collect_all_ptrs(subscription_ptrs, timer_ptrs, service_ptrs, client_ptrs, waitable_ptrs);
    trigger_ptr = cbg.get_notify_guard_condition();
  }

  std::vector<rclcpp::SubscriptionBase::WeakPtr> subscription_ptrs;
  std::vector<rclcpp::TimerBase::WeakPtr> timer_ptrs;
  std::vector<rclcpp::ServiceBase::WeakPtr> service_ptrs;
  std::vector<rclcpp::ClientBase::WeakPtr> client_ptrs;
  std::vector<rclcpp::Waitable::WeakPtr> waitable_ptrs;

  rclcpp::GuardCondition::WeakPtr trigger_ptr;
};
}
