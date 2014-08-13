#ifndef __rclcpp__executor__SingleThreadExecutor__h__
#define __rclcpp__executor__SingleThreadExecutor__h__

#include <algorithm>
#include <cassert>
#include <iostream>
#include <vector>

#include "rclcpp/Node.h"
#include "ros_middleware_interface/functions.h"
#include "ros_middleware_interface/handles.h"

namespace rclcpp
{

namespace executor
{

class SingleThreadExecutor
{
public:
  SingleThreadExecutor() {}

  void register_node(rclcpp::Node *node)
  {
    this->nodes_.push_back(node);
  }

  void unregister_node(rclcpp::Node *node)
  {
    auto it = std::find(this->nodes_.begin(), this->nodes_.end(), node);
    if (it != this->nodes_.end())
    {
      this->nodes_.erase(it);
    }
  }

  void exec()
  {
    while (1)
    {
      size_t total_subscribers_size = 0;
      for (auto node : this->nodes_)
      {
        total_subscribers_size += node->subscribers_.size();
      }

      ros_middleware_interface::SubscriberHandles subscriber_handles;
      subscriber_handles.subscriber_count_ = total_subscribers_size;
      subscriber_handles.subscribers_ = static_cast<void **>(malloc(sizeof(void *) * total_subscribers_size));

      size_t handles_index = 0;
      for (auto node : this->nodes_) {
        for (auto subscriber : node->subscribers_)
        {
          assert(handles_index < total_subscribers_size);
          subscriber_handles.subscribers_[handles_index] = subscriber->subscriber_handle_.data_;
          handles_index += 1;
        }
      }

      ros_middleware_interface::GuardConditionHandles guard_condition_handles;
      guard_condition_handles.guard_condition_count_ = 0;
      guard_condition_handles.guard_conditions_ = 0;

      ros_middleware_interface::wait(subscriber_handles, guard_condition_handles);

      for (size_t index = 0; index < total_subscribers_size; ++index)
      {
        void *handle = subscriber_handles.subscribers_[index];
        if (!handle)
        {
          continue;
        }
        for (auto node : this->nodes_)
        {
          for (auto subscriber : node->subscribers_)
          {
            if (subscriber->subscriber_handle_.data_ == handle)
            {
              // Do callback
              std::cout << "Callback for subscriber of topic: " << subscriber->topic_name_ << std::endl;
              void * ros_msg = subscriber->create_message();
              bool taken = ros_middleware_interface::take(subscriber->subscriber_handle_, ros_msg);
              if (taken)
              {
                std::cout << "- received message on topic: " << subscriber->topic_name_ << std::endl;
                subscriber->handle_message(ros_msg);
              }
              subscriber->delete_message(ros_msg);
            }
          }
        }
      }
    }
  }

private:
  SingleThreadExecutor(const SingleThreadExecutor&);
  std::vector<rclcpp::Node *> nodes_;

};

} /* namespace executor */

} /* namespace rclcpp */

#endif  // __rclcpp__executor__SingleThreadExecutor__h__
