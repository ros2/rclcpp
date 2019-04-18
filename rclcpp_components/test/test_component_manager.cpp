// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>

#include "component_manager.hpp"

#include "rcpputils/filesystem_helper.hpp"

class TestComponentManager : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }
};

TEST_F(TestComponentManager, get_component_resources_invalid)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  EXPECT_THROW(manager->get_component_resources("invalid_package"),
    rclcpp_components::ComponentManagerException);
}

TEST_F(TestComponentManager, get_component_resources_valid)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  auto resources = manager->get_component_resources("rclcpp_components");
  EXPECT_EQ(3u, resources.size());

  EXPECT_EQ("test_rclcpp_components::TestComponentFoo", resources[0].first);
  EXPECT_EQ("test_rclcpp_components::TestComponentBar", resources[1].first);
  EXPECT_EQ("test_rclcpp_components::TestComponentNoNode", resources[2].first);

  EXPECT_TRUE(rcpputils::fs::path(resources[0].second).exists());
  EXPECT_TRUE(rcpputils::fs::path(resources[1].second).exists());
  EXPECT_TRUE(rcpputils::fs::path(resources[2].second).exists());
}

TEST_F(TestComponentManager, create_component_factory_valid)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  auto resources = manager->get_component_resources("rclcpp_components");
  EXPECT_EQ(3u, resources.size());

  // Repeated loading should reuse existing class loader and not throw.
  EXPECT_NO_THROW(auto factory = manager->create_component_factory(resources[0]););
  EXPECT_NO_THROW(auto factory = manager->create_component_factory(resources[0]););

  for (const auto & resource : resources) {
    auto factory = manager->create_component_factory(resource);
    EXPECT_NE(nullptr, factory);
  }
}

TEST_F(TestComponentManager, create_component_factory_invalid)
{
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  // Test invalid library
  EXPECT_THROW(manager->create_component_factory({"foo_class", "invalid_library.so"}),
    rclcpp_components::ComponentManagerException);

  // Test valid library with invalid class
  auto resources = manager->get_component_resources("rclcpp_components");
  auto factory = manager->create_component_factory({"foo_class", resources[0].second});
  EXPECT_EQ(nullptr, factory);
}
