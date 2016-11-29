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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

static constexpr auto chatter_topic = "lifecycle_chatter";
static constexpr auto notification_topic = "lc_talker__transition_notify";

class LifecycleListener : public rclcpp::node::Node
{
public:
  explicit LifecycleListener(const std::string & node_name)
  : rclcpp::node::Node(node_name)
  {
    sub_data_ = this->create_subscription<std_msgs::msg::String>(chatter_topic,
      std::bind(&LifecycleListener::data_callback, this, std::placeholders::_1));

    sub_notification_ = this->create_subscription<lifecycle_msgs::msg::Transition>(
      notification_topic, std::bind(&LifecycleListener::notification_callback, this,
      std::placeholders::_1));
  }

  void data_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    printf("[%s] data_callback: %s\n", get_name().c_str(), msg->data.c_str());
  }

  void notification_callback(const lifecycle_msgs::msg::Transition::SharedPtr msg)
  {
    printf("[%s] notify callback: Transition from state %d to %d\n",
      get_name().c_str(), static_cast<int>(msg->start_state), static_cast<int>(msg->goal_state));
  }

private:
  std::shared_ptr<rclcpp::subscription::Subscription<std_msgs::msg::String>> sub_data_;
  std::shared_ptr<rclcpp::subscription::Subscription<lifecycle_msgs::msg::Transition>>
  sub_notification_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto lc_listener = std::make_shared<LifecycleListener>("lc_listener");
  rclcpp::spin(lc_listener);

  return 0;
}
