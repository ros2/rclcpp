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

#include <gtest/gtest.h>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/graph_listener.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

constexpr char node_name[] = "node";
constexpr char node_namespace[] = "ns";

}  // namespace

class TestGraphListener : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>(node_name, node_namespace);

    using rclcpp::contexts::get_global_default_context;
    graph_listener_ =
      std::make_shared<rclcpp::graph_listener::GraphListener>(get_global_default_context());
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node() {return node_;}
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener() {return graph_listener_;}

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::graph_listener::GraphListener> graph_listener_;
};

