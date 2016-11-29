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
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_manager.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

// service topics for general lifecycle manager
static constexpr auto manager_get_state_topic = "lifecycle_manager__get_state";
static constexpr auto manager_change_state_topic = "lifecycle_manager__change_state";

// service topics for node-attached services
static const std::string lifecycle_node = "lc_talker";
static const std::string node_get_state_topic = lifecycle_node+"__get_state";
static const std::string node_change_state_topic = lifecycle_node+"__change_state";

class LifecycleServiceClient : public rclcpp::node::Node
{
public:
  explicit LifecycleServiceClient(const std::string & node_name)
  : rclcpp::node::Node(node_name)
  {}

  void
  init()
  {
    // Service clients pointing to a global lifecycle manager
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      manager_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      manager_change_state_topic);

    // Service client pointing to each individual service
    client_get_single_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_single_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_change_state_topic);
  }

  unsigned int
  get_state(const std::string & node_name, std::chrono::seconds time_out = 3_s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    request->node_name = node_name;

    if (!client_get_state_->wait_for_service(time_out)) {
      fprintf(stderr, "Service %s is not available.\n",
          client_get_state_->get_service_name().c_str());
      return static_cast<int>(rclcpp::lifecycle::LifecyclePrimaryStatesT::UNKNOWN);
    }

    auto result = client_get_state_->async_send_request(request);
    // Kind of a hack for making a async request a synchronous one.
    while (result.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(result.get())
    {
      printf("[%s] Node %s has current state %d.\n",
        get_name().c_str(), node_name.c_str(), result.get()->current_state);
      return result.get()->current_state;
    } else {
      fprintf(stderr, "[%s] Failed to get current state for node %s\n",
        get_name().c_str(), node_name.c_str());
      return static_cast<int>(rclcpp::lifecycle::LifecyclePrimaryStatesT::UNKNOWN);
    }
  }

  unsigned int
  get_single_state(std::chrono::seconds time_out = 3_s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_single_state_->wait_for_service(time_out)) {
      fprintf(stderr, "Service %s is not available.\n",
          client_get_single_state_->get_service_name().c_str());
      return static_cast<int>(rclcpp::lifecycle::LifecyclePrimaryStatesT::UNKNOWN);
    }

    auto result = client_get_single_state_->async_send_request(request);
    // Kind of a hack for making a async request a synchronous one.
    while (result.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(result.get())
    {
      printf("[%s] Node %s has current state %d\n",
        get_name().c_str(), lifecycle_node.c_str(), result.get()->current_state);
      return result.get()->current_state;
    } else {
      fprintf(stderr, "[%s] Failed to get current state for node %s\n",
        get_name().c_str(), lifecycle_node.c_str());
      return static_cast<int>(rclcpp::lifecycle::LifecyclePrimaryStatesT::UNKNOWN);
    }
  }

  bool
  change_state(const std::string & node_name, rclcpp::lifecycle::LifecycleTransitionsT transition,
      std::chrono::seconds time_out = 3_s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->node_name = node_name;
    request->transition = static_cast<unsigned int>(transition);

    if (!client_change_state_->wait_for_service(time_out)) {
      fprintf(stderr, "Service %s is not available.\n",
        client_change_state_->get_service_name().c_str());
      return false;
    }

    auto result = client_change_state_->async_send_request(request);
    while (result.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(result.get()->success)
    {
      printf("[%s] Transition %d successfully triggered.\n",
        get_name().c_str(), static_cast<int>(transition));
      return true;
    } else {
      fprintf(stderr, "[%s] Failed to trigger transition %d\n",
        get_name().c_str(), static_cast<int>(transition));
      return false;
    }
  }

  bool
  change_single_state(rclcpp::lifecycle::LifecycleTransitionsT transition,
      std::chrono::seconds time_out = 3_s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->node_name = lifecycle_node;
    request->transition = static_cast<unsigned int>(transition);

    if (!client_change_single_state_->wait_for_service(time_out)) {
      fprintf(stderr, "Service %s is not available.\n",
        client_change_single_state_->get_service_name().c_str());
      return false;
    }

    auto result = client_change_single_state_->async_send_request(request);
    while (result.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(result.get()->success)
    {
      printf("[%s] Transition %d successfully triggered.\n",
        get_name().c_str(), static_cast<int>(transition));
      return true;
    } else {
      fprintf(stderr, "[%s] Failed to trigger transition %d\n",
        get_name().c_str(), static_cast<int>(transition));
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::client::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::client::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  std::shared_ptr<rclcpp::client::Client<lifecycle_msgs::srv::GetState>> client_get_single_state_;
  std::shared_ptr<rclcpp::client::Client<lifecycle_msgs::srv::ChangeState>> client_change_single_state_;
};

void
callee_script(std::shared_ptr<LifecycleServiceClient> lc_client,
  const std::string & node_name)
{
  auto sleep_time = 10_s;

  {  // configure
    lc_client->change_single_state(rclcpp::lifecycle::LifecycleTransitionsT::CONFIGURING);
    lc_client->get_single_state();
  }
  {  // activate single node
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::ACTIVATING);
    lc_client->get_state(node_name);
  }
  {  // deactivate single node
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::DEACTIVATING);
    lc_client->get_state(node_name);
  }
  {  // activate lifecycle_manager
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::ACTIVATING);
    lc_client->get_state(node_name);
  }
  {  // deactivate lifecycle_manager
    std::this_thread::sleep_for(sleep_time);
    lc_client->change_state(node_name, rclcpp::lifecycle::LifecycleTransitionsT::DEACTIVATING);
    lc_client->get_state(node_name);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");
  lc_client->init();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(lc_client);

  std::shared_future<void> script = std::async(std::launch::async,
      std::bind(callee_script, lc_client, lifecycle_node));
  exe.spin_until_future_complete(script);

  return 0;
}
