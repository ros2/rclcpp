// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

class TestStateWrapper : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
  }
};

TEST_F(TestStateWrapper, wrapper) {
  {
    rclcpp_lifecycle::State state(1, "my_state");
    EXPECT_EQ(1, state.id());
    EXPECT_FALSE(state.label().empty());
    EXPECT_STREQ("my_state", state.label().c_str());
  }

  {
    rcl_lifecycle_state_t lc_state = {"my_c_state", 2, NULL, NULL, 0};
    rclcpp_lifecycle::State c_state(lc_state.id, lc_state.label);
    EXPECT_EQ(2, c_state.id());
    EXPECT_FALSE(c_state.label().empty());
    EXPECT_STREQ("my_c_state", c_state.label().c_str());
  }

  {
    rcl_lifecycle_state_t lc_state = {"my_c_state", 2, NULL, NULL, 0};
    rclcpp_lifecycle::State c_state(&lc_state);
    EXPECT_EQ(2, c_state.id());
    EXPECT_FALSE(c_state.label().empty());
    EXPECT_STREQ("my_c_state", c_state.label().c_str());
  }

  {
   rcl_lifecycle_state_t * lc_state 
     = new rcl_lifecycle_state_t {"my_c_state", 3, NULL, NULL, 0};
   rclcpp_lifecycle::State c_state(lc_state->id, lc_state->label);
   EXPECT_EQ(3, c_state.id());
   EXPECT_FALSE(c_state.label().empty());
   EXPECT_STREQ("my_c_state", c_state.label().c_str());
  }


  // introduces flakiness
  // unsupported behavior!
  /*
  {
   rcl_lifecycle_state_t * lc_state 
     = new rcl_lifecycle_state_t {"my_c_state", 3, NULL, NULL, 0};
   rclcpp_lifecycle::State c_state(lc_state);
   delete lc_state;
   lc_state = NULL;
   EXPECT_EQ(3, c_state.id());
   EXPECT_STREQ("my_c_state", c_state.label().c_str());
  }
  */
}
