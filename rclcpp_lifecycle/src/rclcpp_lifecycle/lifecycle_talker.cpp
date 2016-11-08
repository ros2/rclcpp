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

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_manager.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"
#include "rclcpp_lifecycle/msg/transition.hpp"
#include "rclcpp_lifecycle/srv/get_state.hpp"
#include "rclcpp_lifecycle/srv/change_state.hpp"

class LifecycleTalker : public rclcpp::node::lifecycle::LifecycleNode
{
public:
  explicit LifecycleTalker(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp::node::lifecycle::LifecycleNode(node_name, intra_process_comms)
  {
    msg_ = std::make_shared<std_msgs::msg::String>();

    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter");
    timer_ = this->get_communication_interface()->create_wall_timer(
      1_s, std::bind(&LifecycleTalker::publish, this));
  }

  void publish()
  {
    static size_t count = 0;
    msg_->data = "Lifecycle HelloWorld #" + std::to_string(++count);
    pub_->publish(msg_);
  }

  bool on_configure()
  {
    printf("%s is going to configure its node\n", get_name().c_str());
    return true;
  }

  bool on_activate()
  {
    rclcpp::node::lifecycle::LifecycleNode::enable_communication();
    printf("%s is going to activate its node\n", get_name().c_str());
    return true;
  }

  bool on_deactivate()
  {
    rclcpp::node::lifecycle::LifecycleNode::disable_communication();
    printf("%s is going to deactivate its node\n", get_name().c_str());
    return true;
  }

private:
  std::shared_ptr<std_msgs::msg::String> msg_;
  std::shared_ptr<rclcpp::publisher::LifecyclePublisher<std_msgs::msg::String>> pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class LifecycleListener : public rclcpp::node::Node
{
public:
  explicit LifecycleListener(const std::string & node_name)
  : rclcpp::node::Node(node_name)
  {
    sub_data_ = this->create_subscription<std_msgs::msg::String>("lifecycle_chatter",
        std::bind(&LifecycleListener::data_callback, this, std::placeholders::_1));
    sub_notification_ = this->create_subscription<rclcpp_lifecycle::msg::Transition>(
      "lifecycle_manager__lc_talker", std::bind(&LifecycleListener::notification_callback, this,
      std::placeholders::_1));
  }

  void data_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "I heard data: [" << msg->data << "]" << std::endl;
  }

  void notification_callback(const rclcpp_lifecycle::msg::Transition::SharedPtr msg)
  {
    std::cout << "Transition triggered:: [ Going from state " <<
      static_cast<int>(msg->start_state) << " to state " <<
      static_cast<int>(msg->goal_state) << " ] " << std::endl;
  }

private:
  std::shared_ptr<rclcpp::subscription::Subscription<std_msgs::msg::String>> sub_data_;
  std::shared_ptr<rclcpp::subscription::Subscription<rclcpp_lifecycle::msg::Transition>>
  sub_notification_;
};

class LifecycleServiceClient : public rclcpp::node::Node
{
public:
  explicit LifecycleServiceClient(const std::string & node_name)
  : rclcpp::node::Node(node_name)
  {}

  void
  init()
  {
    client_get_state_ = this->create_client<rclcpp_lifecycle::srv::GetState>(
      "lifecycle_manager__get_state");
    client_get_single_state_ = this->create_client<rclcpp_lifecycle::srv::GetState>(
      "lc_talker__get_state");
    client_change_state_ = this->create_client<rclcpp_lifecycle::srv::ChangeState>(
      "lifecycle_manager__change_state");
    client_change_single_state_ = this->create_client<rclcpp_lifecycle::srv::ChangeState>(
      "lc_talker__change_state");
  }

  unsigned int
  get_state(const std::string & node_name)
  {
    auto request = std::make_shared<rclcpp_lifecycle::srv::GetState::Request>();
    request->node_name = node_name;

    if (!client_get_state_->wait_for_service(3_s)) {
      fprintf(stderr, "Service %s is not available.\n", client_get_state_->get_service_name().c_str());
      return static_cast<int>(rclcpp::lifecycle::LifecyclePrimaryStatesT::UNKNOWN);
    }

    auto result = client_get_state_->async_send_request(request);
    fprintf(stderr, "Asking current state of node %s. Let's wait!\n", node_name.c_str());
    // Kind of a hack for making a async request a synchronous one.
    while (result.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return result.get()->current_state;
  }

  unsigned int
  get_single_state()
  {
    auto request = std::make_shared<rclcpp_lifecycle::srv::GetState::Request>();

    if (!client_get_single_state_->wait_for_service(3_s)) {
      fprintf(stderr, "Service %s is not available.\n", client_get_single_state_->get_service_name().c_str());
      return static_cast<int>(rclcpp::lifecycle::LifecyclePrimaryStatesT::UNKNOWN);
    }

    auto result = client_get_single_state_->async_send_request(request);
    fprintf(stderr, "Asking single state of node %s. Let's wait!\n", "lc_talker");
    // Kind of a hack for making a async request a synchronous one.
    while (result.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return result.get()->current_state;
  }

  bool
  change_state(const std::string & node_name, rclcpp::lifecycle::LifecycleTransitionsT transition)
  {
    auto request = std::make_shared<rclcpp_lifecycle::srv::ChangeState::Request>();
    request->node_name = node_name;
    request->transition = static_cast<unsigned int>(transition);

    if (!client_change_state_->wait_for_service(3_s)) {
      fprintf(stderr, "Service %s is not available.\n", client_change_state_->get_service_name().c_str());
      return false;
    }

    auto result = client_change_state_->async_send_request(request);
    fprintf(stderr, "Going to trigger transition %u for node %s\n", request->transition,
      node_name.c_str());
    while (result.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return result.get()->success;
  }

  bool
  change_single_state(rclcpp::lifecycle::LifecycleTransitionsT transition)
  {
    auto request = std::make_shared<rclcpp_lifecycle::srv::ChangeState::Request>();
    request->transition = static_cast<unsigned int>(transition);

    if (!client_change_single_state_->wait_for_service(3_s)) {
      fprintf(stderr, "Service %s is not available.\n", client_change_single_state_->get_service_name().c_str());
      return false;
    }

    auto result = client_change_single_state_->async_send_request(request);
    fprintf(stderr, "Going to trigger transition %u for node %s\n", request->transition,
      "lc_talker");
    while (result.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return result.get()->success;
  }

private:
  std::shared_ptr<rclcpp::client::Client<rclcpp_lifecycle::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::client::Client<rclcpp_lifecycle::srv::ChangeState>> client_change_state_;
  std::shared_ptr<rclcpp::client::Client<rclcpp_lifecycle::srv::GetState>> client_get_single_state_;
  std::shared_ptr<rclcpp::client::Client<rclcpp_lifecycle::srv::ChangeState>> client_change_single_state_;
};

void
callee_script(std::shared_ptr<LifecycleServiceClient> lc_client,
  const std::string & node_name)
{
  auto sleep_time = 2_s;

    std::this_thread::sleep_for(sleep_time);
  /*
  {  // configure
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::CONFIGURING);
    //lc_client->change_single_state(rclcpp::lifecycle::LifecycleTransitionsT::CONFIGURING);
    //auto current_state = lc_client->get_single_state();
    auto current_state = lc_client->get_state(node_name);
    printf("Node %s is in state %u\n", "lc_talker", current_state);
  }
  {  // activate
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::ACTIVATING);
    auto current_state = lc_client->get_state(node_name);
    printf("Node %s is in state %u\n", node_name.c_str(), current_state);
  }
  {  // deactivate
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::DEACTIVATING);
    auto current_state = lc_client->get_state(node_name);
    printf("Node %s is in state %u\n", node_name.c_str(), current_state);
  }
  {  // activate againn
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::ACTIVATING);
    auto current_state = lc_client->get_state(node_name);
    printf("Node %s is in state %u\n", node_name.c_str(), current_state);
  }
  {  // deactivate again
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::DEACTIVATING);
    auto current_state = lc_client->get_state(node_name);
    printf("Node %s is in state %u\n", node_name.c_str(), current_state);
  }
  */
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<LifecycleTalker> lc_node =
    std::make_shared<LifecycleTalker>("lc_talker");

  std::shared_ptr<LifecycleListener> lc_listener =
    std::make_shared<LifecycleListener>("lc_listener");

  std::shared_ptr<LifecycleServiceClient> lc_client =
    std::make_shared<LifecycleServiceClient>("lc_client");
  lc_client->init();

  rclcpp::lifecycle::LifecycleManager lm;
  lm.add_node_interface(lc_node);

  exe.add_node(lc_node->get_communication_interface());
  //exe.add_node(lc_listener);
  //exe.add_node(lc_client);
  exe.add_node(lm.get_node_base_interface());

  auto node_name = lc_node->get_base_interface()->get_name();

  std::shared_future<void> script = std::async(std::launch::async,
      std::bind(callee_script, lc_client, node_name));

  exe.spin_until_future_complete(script);

  return 0;
}
