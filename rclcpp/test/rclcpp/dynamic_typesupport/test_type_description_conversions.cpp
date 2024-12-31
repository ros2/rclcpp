// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/dynamic_typesupport/type_description_conversions.hpp"
#include "rosidl_runtime_cpp/type_description/individual_type_description__struct.hpp"
#include "rosidl_runtime_cpp/type_description/type_description__struct.hpp"
#include "rosidl_runtime_cpp/type_description/type_source__struct.hpp"
#include "type_description_interfaces/msg/individual_type_description.hpp"
#include "type_description_interfaces/msg/type_description.hpp"
#include "type_description_interfaces/msg/type_source.hpp"

using rclcpp::dynamic_typesupport::
convert_individual_type_description_msg_to_runtime;
using rclcpp::dynamic_typesupport::
convert_individual_type_description_runtime_to_msg;
using rclcpp::dynamic_typesupport::convert_type_description_msg_to_runtime;
using rclcpp::dynamic_typesupport::convert_type_description_runtime_to_msg;
using rclcpp::dynamic_typesupport::convert_type_source_sequence_msg_to_runtime;
using rclcpp::dynamic_typesupport::convert_type_source_sequence_runtime_to_msg;

TEST(TestTypeDescriptionConversions, individual_type_description_roundtrip)
{
  rosidl_runtime_cpp::type_description::IndividualTypeDescription original;
  original.type_name = "test_type_name";

  original.fields.emplace_back();
  original.fields.back().name = "field1";
  original.fields.back().type.type_id = 1;

  original.fields.emplace_back();
  original.fields.back().name = "field2";
  original.fields.back().type.type_id = 2;

  type_description_interfaces::msg::IndividualTypeDescription msg =
    convert_individual_type_description_runtime_to_msg(original);
  rosidl_runtime_cpp::type_description::IndividualTypeDescription converted_back
    =
    convert_individual_type_description_msg_to_runtime(msg);

  EXPECT_EQ(original.type_name, converted_back.type_name);
  ASSERT_EQ(original.fields.size(), converted_back.fields.size());
  for (size_t i = 0; i < original.fields.size(); ++i) {
    EXPECT_EQ(original.fields[i].name, converted_back.fields[i].name);
    EXPECT_EQ(original.fields[i].type, converted_back.fields[i].type);
  }
}

TEST(TestTypeDescriptionConversions, type_description_roundtrip)
{
  rosidl_runtime_cpp::type_description::TypeDescription original;
  original.type_description.type_name = "main_type";
  original.type_description.fields.emplace_back();
  original.type_description.fields.back().name = "field1";
  original.type_description.fields.back().type.type_id = 1;
  original.type_description.fields.emplace_back();
  original.type_description.fields.back().name = "field2";
  original.type_description.fields.back().type.type_id = 2;

  rosidl_runtime_cpp::type_description::IndividualTypeDescription ref1;
  ref1.type_name = "ref_type_1";
  ref1.fields.emplace_back();
  ref1.fields.back().name = "ref_field1";
  ref1.fields.back().type.type_id = 1;

  rosidl_runtime_cpp::type_description::IndividualTypeDescription ref2;
  ref2.type_name = "ref_type_2";
  ref2.fields.emplace_back();
  ref2.fields.back().name = "ref_field2";
  ref2.fields.back().type.type_id = 2;

  original.referenced_type_descriptions.push_back(ref1);
  original.referenced_type_descriptions.push_back(ref2);

  type_description_interfaces::msg::TypeDescription msg =
    convert_type_description_runtime_to_msg(original);
  rosidl_runtime_cpp::type_description::TypeDescription converted_back =
    convert_type_description_msg_to_runtime(msg);

  EXPECT_EQ(original.type_description.type_name,
    converted_back.type_description.type_name);
  ASSERT_EQ(original.referenced_type_descriptions.size(),
    converted_back.referenced_type_descriptions.size());

  for (size_t i = 0; i < original.referenced_type_descriptions.size(); ++i) {
    EXPECT_EQ(original.referenced_type_descriptions[i].type_name,
      converted_back.referenced_type_descriptions[i].type_name);
  }
}

TEST(TestTypeSourceConversions, type_source_roundtrip)
{
  rosidl_runtime_cpp::type_description::TypeSource original;
  original.type_name = "test_type_name";
  original.encoding = "test_encoding";
  original.raw_file_contents = "test_contents";

  type_description_interfaces::msg::TypeSource msg =
    convert_type_source_sequence_runtime_to_msg(original);
  rosidl_runtime_cpp::type_description::TypeSource converted_back =
    convert_type_source_sequence_msg_to_runtime(msg);

  EXPECT_EQ(original.type_name, converted_back.type_name);
  EXPECT_EQ(original.encoding, converted_back.encoding);
  EXPECT_EQ(original.raw_file_contents, converted_back.raw_file_contents);
}
