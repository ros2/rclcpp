^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_action
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


16.0.5 (2023-07-17)
-------------------

16.0.4 (2023-04-25)
-------------------
* Revert "Revert "extract the result response before the callback is issued. (`#2133 <https://github.com/ros2/rclcpp/issues/2133>`_)" (`#2148 <https://github.com/ros2/rclcpp/issues/2148>`_)" (`#2152 <https://github.com/ros2/rclcpp/issues/2152>`_)
* Revert "extract the result response before the callback is issued. (`#2133 <https://github.com/ros2/rclcpp/issues/2133>`_)" (`#2148 <https://github.com/ros2/rclcpp/issues/2148>`_)
* extract the result response before the callback is issued. (`#2133 <https://github.com/ros2/rclcpp/issues/2133>`_)
* Contributors: Tomoya Fujita

16.0.3 (2023-01-10)
-------------------

16.0.2 (2022-11-07)
-------------------

16.0.1 (2022-04-13)
-------------------

16.0.0 (2022-04-08)
-------------------
* remove things that were deprecated during galactic (`#1913 <https://github.com/ros2/rclcpp/issues/1913>`_)
* Contributors: William Woodall

15.4.0 (2022-04-05)
-------------------
* add take_data_by_entity_id API to waitable (`#1892 <https://github.com/ros2/rclcpp/issues/1892>`_)
* Contributors: Alberto Soragna

15.3.0 (2022-03-30)
-------------------

15.2.0 (2022-03-24)
-------------------
* Fix rosdoc2 issues (`#1897 <https://github.com/ros2/rclcpp/issues/1897>`_)
* Contributors: Chris Lalancette

15.1.0 (2022-03-01)
-------------------
* Add RMW listener APIs (`#1579 <https://github.com/ros2/rclcpp/issues/1579>`_)
* Install headers to include/${PROJECT_NAME} (`#1888 <https://github.com/ros2/rclcpp/issues/1888>`_)
* Contributors: Shane Loretz, iRobot ROS

15.0.0 (2022-01-14)
-------------------
* Fix include order and relative paths for cpplint (`#1859 <https://github.com/ros2/rclcpp/issues/1859>`_)
* Contributors: Jacob Perron

14.1.0 (2022-01-05)
-------------------

14.0.0 (2021-12-17)
-------------------
* Fixes for uncrustify 0.72 (`#1844 <https://github.com/ros2/rclcpp/issues/1844>`_)
* Use rclcpp::guard_condition (`#1612 <https://github.com/ros2/rclcpp/issues/1612>`_)
* Remove author by request (`#1818 <https://github.com/ros2/rclcpp/issues/1818>`_)
* Update maintainers (`#1817 <https://github.com/ros2/rclcpp/issues/1817>`_)
* Suppress clang dead-store warnings in the benchmarks. (`#1802 <https://github.com/ros2/rclcpp/issues/1802>`_)
* Contributors: Chris Lalancette, Jacob Perron, mauropasse

13.1.0 (2021-10-18)
-------------------
* Deprecate the `void shared_ptr<MessageT>` subscription callback signatures (`#1713 <https://github.com/ros2/rclcpp/issues/1713>`_)
* Contributors: Abrar Rahman Protyasha, Tomoya Fujita

13.0.0 (2021-08-23)
-------------------
* Use rcpputils/scope_exit.hpp and remove rclcpp/scope_exit.hpp. (`#1727 <https://github.com/ros2/rclcpp/issues/1727>`_)
* Contributors: Christophe Bedard

12.0.0 (2021-07-26)
-------------------

11.2.0 (2021-07-21)
-------------------

11.1.0 (2021-07-13)
-------------------
* Fixed occasionally missing goal result caused by race condition (`#1677 <https://github.com/ros2/rclcpp/issues/1677>`_)
* Contributors: Kaven Yau

11.0.0 (2021-05-18)
-------------------
* Bump the benchmark timeout for benchmark_action_client (`#1671 <https://github.com/ros2/rclcpp/issues/1671>`_)
* Contributors: Scott K Logan

10.0.0 (2021-05-11)
-------------------
* Returns CancelResponse::REJECT while goal handle failed to transit to CANCELING state (`#1641 <https://github.com/ros2/rclcpp/issues/1641>`_)
* Fix action server deadlock issue that caused by other mutexes locked in CancelCallback (`#1635 <https://github.com/ros2/rclcpp/issues/1635>`_)
* Contributors: Kaven Yau

9.0.2 (2021-04-14)
------------------

9.0.1 (2021-04-12)
------------------

9.0.0 (2021-04-06)
------------------
* updating quality declaration links (re: `ros2/docs.ros2.org#52 <https://github.com/ros2/docs.ros2.org/issues/52>`_) (`#1615 <https://github.com/ros2/rclcpp/issues/1615>`_)
* Contributors: shonigmann

8.2.0 (2021-03-31)
------------------

8.1.0 (2021-03-25)
------------------

8.0.0 (2021-03-23)
------------------

7.0.1 (2021-03-22)
------------------

7.0.0 (2021-03-18)
------------------
* Add support for rmw_connextdds (`#1574 <https://github.com/ros2/rclcpp/issues/1574>`_)
* node_handle must be destroyed after client_handle to prevent memory leak (`#1562 <https://github.com/ros2/rclcpp/issues/1562>`_)
* Contributors: Andrea Sorbini, Tomoya Fujita

6.3.1 (2021-02-08)
------------------
* Finalize rcl_handle to prevent leak (`#1528 <https://github.com/ros2/rclcpp/issues/1528>`_) (`#1529 <https://github.com/ros2/rclcpp/issues/1529>`_)
* Fix `#1526 <https://github.com/ros2/rclcpp/issues/1526>`_. (`#1527 <https://github.com/ros2/rclcpp/issues/1527>`_)
* Contributors: y-okumura-isp

6.3.0 (2021-01-25)
------------------
* Fix action server deadlock (`#1285 <https://github.com/ros2/rclcpp/issues/1285>`_) (`#1313 <https://github.com/ros2/rclcpp/issues/1313>`_)
* Contributors: Daisuke Sato

6.2.0 (2021-01-08)
------------------
* Goal response callback compatibility shim with deprecation of old signature (`#1495 <https://github.com/ros2/rclcpp/issues/1495>`_)
* [rclcpp_action] Add warnings (`#1405 <https://github.com/ros2/rclcpp/issues/1405>`_)
* Contributors: Audrow Nash, Ivan Santiago Paunovic

6.1.0 (2020-12-10)
------------------
* Update QDs to QL 1 (`#1477 <https://github.com/ros2/rclcpp/issues/1477>`_)
* Contributors: Stephen Brawner

6.0.0 (2020-11-18)
------------------
* Add `take_data` to `Waitable` and `data` to `AnyExecutable` (`#1241 <https://github.com/ros2/rclcpp/issues/1241>`_)
* Fix test crashes on CentOS 7 (`#1449 <https://github.com/ros2/rclcpp/issues/1449>`_)
* Bump rclcpp packages to Quality Level 2 (`#1445 <https://github.com/ros2/rclcpp/issues/1445>`_)
* Add rclcpp_action action_server benchmarks (`#1433 <https://github.com/ros2/rclcpp/issues/1433>`_)
* Contributors: Audrow Nash, Chris Lalancette, Louise Poubel, brawner

5.1.0 (2020-11-02)
------------------
* Benchmark rclcpp_action action_client (`#1429 <https://github.com/ros2/rclcpp/issues/1429>`_)
* Add missing locking to the rclcpp_action::ServerBase. (`#1421 <https://github.com/ros2/rclcpp/issues/1421>`_)
* Increase test timeouts of slow running tests with rmw_connext_cpp (`#1400 <https://github.com/ros2/rclcpp/issues/1400>`_)
* Update maintainers (`#1384 <https://github.com/ros2/rclcpp/issues/1384>`_)
* Increase coverage rclcpp_action to 95% (`#1290 <https://github.com/ros2/rclcpp/issues/1290>`_)
* Contributors: Chris Lalancette, Ivan Santiago Paunovic, brawner

5.0.0 (2020-09-18)
------------------
* Pass goal handle to goal response callback instead of a future (`#1311 <https://github.com/ros2/rclcpp/issues/1311>`_)
* Remove deprecated client goal handle method for getting result (`#1309 <https://github.com/ros2/rclcpp/issues/1309>`_)
* Increase test timeout necessary for Connext (`#1256 <https://github.com/ros2/rclcpp/issues/1256>`_)
* Contributors: Dirk Thomas, Jacob Perron

4.0.0 (2020-07-09)
------------------
* Bump to QD to level 3 and fixed links (`#1158 <https://github.com/ros2/rclcpp/issues/1158>`_)
* Contributors: Alejandro Hernández Cordero

3.0.0 (2020-06-18)
------------------
* Add rcl_action_client_options when creating action client. (`#1133 <https://github.com/ros2/rclcpp/issues/1133>`_)
* Fix doxygen warnings (`#1163 <https://github.com/ros2/rclcpp/issues/1163>`_)
* Increase rclcpp_action test coverage (`#1153 <https://github.com/ros2/rclcpp/issues/1153>`_)
* Contributors: Alejandro Hernández Cordero, Michel Hidalgo, tomoya

2.0.0 (2020-06-01)
------------------
* Added missing virtual destructors. (`#1149 <https://github.com/ros2/rclcpp/issues/1149>`_)
* Add Security Vulnerability Policy pointing to REP-2006. (`#1130 <https://github.com/ros2/rclcpp/issues/1130>`_)
* Contributors: Chris Lalancette, Ivan Santiago Paunovic

1.1.0 (2020-05-26)
------------------
* Action client holds weak pointers to goal handles (`#1122 <https://github.com/ros2/rclcpp/issues/1122>`_)
* Deprecate ClientGoalHandle::async_result() (`#1120 <https://github.com/ros2/rclcpp/issues/1120>`_)
* Improve documentation (`#1106 <https://github.com/ros2/rclcpp/issues/1106>`_)
* Fixed rep links and added more details to dependencies in quality declaration (`#1116 <https://github.com/ros2/rclcpp/issues/1116>`_)
* Update quality declaration to reflect version 1.0 (`#1115 <https://github.com/ros2/rclcpp/issues/1115>`_)
* Contributors: Alejandro Hernández Cordero, Jacob Perron, Stephen Brawner

1.0.0 (2020-05-12)
------------------

0.9.1 (2020-05-08)
------------------
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
