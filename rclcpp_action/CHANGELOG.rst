^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_action
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Forthcoming
-----------
* Added Quality declaration: rclcpp, rclpp_action, rclcpp_components andrclcpp_lifecycle (`#1100 <https://github.com/ros2/rclcpp/issues/1100>`_)
* Contributors: Alejandro Hernández Cordero

0.9.0 (2020-04-29)
------------------
* Increasing test coverage of rclcpp_action (`#1043 <https://github.com/ros2/rclcpp/issues/1043>`_)
* Export targets in addition to include directories / libraries (`#1096 <https://github.com/ros2/rclcpp/issues/1096>`_)
* Deprecate redundant namespaces (`#1083 <https://github.com/ros2/rclcpp/issues/1083>`_)
* Rename rosidl_generator_c namespace to rosidl_runtime_c (`#1062 <https://github.com/ros2/rclcpp/issues/1062>`_)
* Changed rosidl_generator_c/cpp to rosidl_runtime_c/cpp (`#1014 <https://github.com/ros2/rclcpp/issues/1014>`_)
* Fix unknown macro errors reported by cppcheck 1.90 (`#1000 <https://github.com/ros2/rclcpp/issues/1000>`_)
* Removed rosidl_generator_c dependency (`#992 <https://github.com/ros2/rclcpp/issues/992>`_)
* Fix typo in action client logger name (`#937 <https://github.com/ros2/rclcpp/issues/937>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Jacob Perron, Stephen Brawner, William Woodall

0.8.3 (2019-11-19)
------------------
* issue-919 Fixed "memory leak" in action clients (`#920 <https://github.com/ros2/rclcpp/issues/920>`_)
* Contributors: astere-cpr

0.8.2 (2019-11-18)
------------------
* Increased a timeout for the ``test_client`` tests. (`#917 <https://github.com/ros2/rclcpp/issues/917>`_)
* Contributors: Michel Hidalgo

0.8.1 (2019-10-23)
------------------
* Template node type for rclcpp action server and clients (`#892 <https://github.com/ros2/rclcpp/issues/892>`_)
* Trait tests for generated actions (`#853 <https://github.com/ros2/rclcpp/issues/853>`_)
* Do not throw exception in action client if take fails (`#888 <https://github.com/ros2/rclcpp/issues/888>`_)
* Contributors: Jacob Perron, Michael Carroll, Steven Macenski

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
