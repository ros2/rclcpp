^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_action
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.16 (2021-05-21)
-------------------

0.7.15 (2020-11-24)
-------------------
* Fix typo in action client logger name. (`#1414 <https://github.com/ros2/rclcpp/issues/1414>`_)
* Contributors: Seulbae Kim

0.7.14 (2020-07-10)
-------------------
* Fixed doxygen warnings. (`#1208 <https://github.com/ros2/rclcpp/issues/1208>`_)
* Contributors: Alejandro Hernández Cordero

0.7.13 (2020-03-12)
-------------------

0.7.12 (2019-12-05)
-------------------
* Fixed memory leak in action clients (`#934 <https://github.com/ros2/rclcpp/issues/934>`_)
* Do not throw exception in action client if take fails (`#891 <https://github.com/ros2/rclcpp/issues/891>`_)
* Contributors: Ivan Santiago Paunovic, Jacob Perron

0.7.11 (2019-10-11)
-------------------

0.7.10 (2019-09-23)
-------------------

0.7.9 (2019-09-20)
------------------

0.7.8 (2019-09-06)
------------------

0.7.7 (2019-07-31)
------------------

0.7.6 (2019-06-12)
------------------

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
