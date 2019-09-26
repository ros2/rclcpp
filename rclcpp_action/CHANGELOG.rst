^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_action
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


0.8.0 (2019-09-26)
------------------
* Fix UnknownGoalHandle error string. (`#856 <https://github.com/ros2/rclcpp/issues/856>`_)
* Guard against making multiple result requests for a goal handle (`#808 <https://github.com/ros2/rclcpp/issues/808>`_)
* Add line break after first open paren in multiline function call (`#785 <https://github.com/ros2/rclcpp/issues/785>`_)
* Fix typo in test fixture tear down method name (`#787 <https://github.com/ros2/rclcpp/issues/787>`_)
* Contributors: Chris Lalancette, Dan Rose, Jacob Perron

0.7.5 (2019-05-30)
------------------

0.7.4 (2019-05-29)
------------------
* Guard against calling null goal response callback (`#738 <https://github.com/ros2/rclcpp/issues/738>`_)
* Contributors: Jacob Perron

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------
* Added return code to CancelGoal service response. (`#710 <https://github.com/ros2/rclcpp/issues/710>`_)
* Contributors: Jacob Perron, William Woodall

0.7.1 (2019-04-26)
------------------
* Added optional callbacks to action client for goal, response, and result. (`#701 <https://github.com/ros2/rclcpp/issues/701>`_)
* Added overload for node interfaces. (`#700 <https://github.com/ros2/rclcpp/issues/700>`_)
* Renamed action state transitions. (`#677 <https://github.com/ros2/rclcpp/issues/677>`_)
* Contributors: Jacob Perron, Karsten Knese

0.7.0 (2019-04-14)
------------------
* Fixed hard-coded duration type representation so int64_t isn't assumed. (`#648 <https://github.com/ros2/rclcpp/issues/648>`_)
* Added documentation to rclcpp_action. (`#650 <https://github.com/ros2/rclcpp/pull/650>`_)
* Updated to use separated action types. (`#601 <https://github.com/ros2/rclcpp/issues/601>`_)
* Updated to wait for action server before sending goal. (`#637 <https://github.com/ros2/rclcpp/issues/637>`_)
* Refactored server goal handle's try_canceling() function. (`#603 <https://github.com/ros2/rclcpp/issues/603>`_)
* Contributors: Emerson Knapp, Jacob Perron, Michel Hidalgo, Shane Loretz

0.6.2 (2018-12-13)
------------------

0.6.1 (2018-12-07)
------------------
* Added wait_for_action_server() for action clients (`#598 <https://github.com/ros2/rclcpp/issues/598>`_)
* Updated to adapt to action implicit changes (`#602 <https://github.com/ros2/rclcpp/issues/602>`_)
* Added rclcpp_action Server implementation (`#593 <https://github.com/ros2/rclcpp/issues/593>`_)
* Added action client implementation (`#594 <https://github.com/ros2/rclcpp/issues/594>`_)
* Added skeleton for Action Server and Client (`#579 <https://github.com/ros2/rclcpp/issues/579>`_)
* Contributors: Michel Hidalgo, Shane Loretz, William Woodall
