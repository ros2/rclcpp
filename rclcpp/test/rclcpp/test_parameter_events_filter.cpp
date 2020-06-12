// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include <string>
#include <memory>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/parameter_events_filter.hpp"

#include "rcl_interfaces/msg/parameter_event.hpp"

class TestParameterEventFilter : public ::testing::Test
{
protected:
  void SetUp()
  {
    empty = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    full = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    multiple = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    np = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    cp = std::make_shared<rcl_interfaces::msg::ParameterEvent>();
    dp = std::make_shared<rcl_interfaces::msg::ParameterEvent>();

    rcl_interfaces::msg::Parameter p;
    p.name = "new";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    p.value.integer_value = 1;
    full->new_parameters.push_back(p);
    np->new_parameters.push_back(p);
    multiple->new_parameters.push_back(p);

    p.name = "new2";
    p.value.integer_value = 2;
    multiple->new_parameters.push_back(p);

    p.name = "changed";
    p.value.integer_value = 1;
    full->changed_parameters.push_back(p);
    cp->changed_parameters.push_back(p);

    p.name = "deleted";
    p.value.integer_value = 1;
    full->deleted_parameters.push_back(p);
    dp->changed_parameters.push_back(p);
  }

  rcl_interfaces::msg::ParameterEvent::SharedPtr empty;
  rcl_interfaces::msg::ParameterEvent::SharedPtr full;
  rcl_interfaces::msg::ParameterEvent::SharedPtr multiple;
  rcl_interfaces::msg::ParameterEvent::SharedPtr np;
  rcl_interfaces::msg::ParameterEvent::SharedPtr cp;
  rcl_interfaces::msg::ParameterEvent::SharedPtr dp;

  rclcpp::ParameterEventsFilter::EventType nt = rclcpp::ParameterEventsFilter::EventType::NEW;
  rclcpp::ParameterEventsFilter::EventType ct = rclcpp::ParameterEventsFilter::EventType::CHANGED;
  rclcpp::ParameterEventsFilter::EventType dt = rclcpp::ParameterEventsFilter::EventType::DELETED;
};

/*
   Testing filters.
 */
TEST_F(TestParameterEventFilter, full_by_type) {
  auto res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed", "deleted"},
    {nt, ct, dt});
  EXPECT_EQ(3u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed", "deleted"},
    {nt, ct});
  EXPECT_EQ(2u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed", "deleted"},
    {nt, dt});
  EXPECT_EQ(2u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed", "deleted"},
    {ct, dt});
  EXPECT_EQ(2u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed", "deleted"},
    {nt});
  EXPECT_EQ(1u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed", "deleted"},
    {ct});
  EXPECT_EQ(1u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed", "deleted"},
    {dt});
  EXPECT_EQ(1u, res.get_events().size());
}

TEST_F(TestParameterEventFilter, full_by_name) {
  auto res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed", "deleted"},
    {nt, ct, dt});
  EXPECT_EQ(3u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "changed"},
    {nt, ct, dt});
  EXPECT_EQ(2u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new", "deleted"},
    {nt, ct, dt});
  EXPECT_EQ(2u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"changed", "deleted"},
    {nt, ct, dt});
  EXPECT_EQ(2u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"new"},
    {nt, ct, dt});
  EXPECT_EQ(1u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"changed"},
    {nt, ct, dt});
  EXPECT_EQ(1u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    full,
    {"deleted"},
    {nt, ct, dt});
  EXPECT_EQ(1u, res.get_events().size());
}

TEST_F(TestParameterEventFilter, empty) {
  auto res = rclcpp::ParameterEventsFilter(
    empty,
    {"new", "changed", "deleted"},
    {nt, ct, dt});
  EXPECT_EQ(0u, res.get_events().size());
}

TEST_F(TestParameterEventFilter, singular) {
  auto res = rclcpp::ParameterEventsFilter(
    np,
    {"new", "changed", "deleted"},
    {nt, ct, dt});
  EXPECT_EQ(1u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    cp,
    {"new", "changed", "deleted"},
    {nt, ct, dt});
  EXPECT_EQ(1u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    dp,
    {"new", "changed", "deleted"},
    {nt, ct, dt});
  EXPECT_EQ(1u, res.get_events().size());
}

TEST_F(TestParameterEventFilter, multiple) {
  auto res = rclcpp::ParameterEventsFilter(
    multiple,
    {"new", "new2"},
    {nt, ct, dt});
  EXPECT_EQ(2u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    multiple,
    {"new2"},
    {nt, ct, dt});
  EXPECT_EQ(1u, res.get_events().size());
  res = rclcpp::ParameterEventsFilter(
    multiple,
    {"new", "new2"},
    {ct, dt});
  EXPECT_EQ(0u, res.get_events().size());
}

TEST_F(TestParameterEventFilter, validate_data) {
  auto res = rclcpp::ParameterEventsFilter(
    multiple,
    {"new", "new2"},
    {nt, ct, dt});
  EXPECT_EQ(2u, res.get_events().size());
  EXPECT_EQ(nt, res.get_events()[0].first);
  EXPECT_EQ(nt, res.get_events()[1].first);
  EXPECT_EQ(1, res.get_events()[0].second->value.integer_value);
  EXPECT_EQ(2, res.get_events()[1].second->value.integer_value);
}
