^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_action
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


29.6.1 (2025-06-23)
-------------------
* Replace std::default_random_engine with std::mt19937 (rolling) (`#2843 <https://github.com/ros2/rclcpp/issues/2843>`_)
* Added missing chrono includes (`#2854 <https://github.com/ros2/rclcpp/issues/2854>`_)
* Contributors: Alejandro Hernández Cordero, keeponoiro

29.6.0 (2025-04-25)
-------------------

29.5.0 (2025-04-18)
-------------------
* Use std::recursive_mutex for action requests. (`#2798 <https://github.com/ros2/rclcpp/issues/2798>`_)
* Contributors: Tomoya Fujita

29.4.0 (2025-04-04)
-------------------
* Remove warning (`#2790 <https://github.com/ros2/rclcpp/issues/2790>`_)
* Harden rclcpp_action::convert(). (`#2786 <https://github.com/ros2/rclcpp/issues/2786>`_)
* Add new interfaces to enable introspection for action (`#2743 <https://github.com/ros2/rclcpp/issues/2743>`_)
* use maybe_unused attribute for the portability. (`#2758 <https://github.com/ros2/rclcpp/issues/2758>`_)
* fix: Expose timers used by rclcpp::Waitables (`#2699 <https://github.com/ros2/rclcpp/issues/2699>`_)
* Collect log messages from rcl, and reset. (`#2720 <https://github.com/ros2/rclcpp/issues/2720>`_)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Janosch Machowinski, Tomoya Fujita

29.3.0 (2024-12-20)
-------------------
* Make ament_cmake a buildtool dependency (`#2689 <https://github.com/ros2/rclcpp/issues/2689>`_)
* Contributors: Nathan Wiebe Neufeldt

29.2.0 (2024-11-25)
-------------------

29.1.0 (2024-11-20)
-------------------
* Fix documentation typo in server_goal_handle.hpp (`#2669 <https://github.com/ros2/rclcpp/issues/2669>`_)
* Contributors: YR

29.0.0 (2024-10-03)
-------------------
* Increase the timeout for the cppcheck on rclcpp_action. (`#2640 <https://github.com/ros2/rclcpp/issues/2640>`_)
* add smart pointer macros definitions to action server and client base classes (`#2631 <https://github.com/ros2/rclcpp/issues/2631>`_)
* Contributors: Alberto Soragna, Chris Lalancette

28.3.3 (2024-07-29)
-------------------

28.3.2 (2024-07-24)
-------------------

28.3.1 (2024-06-25)
-------------------
* Fix typo in function doc (`#2563 <https://github.com/ros2/rclcpp/issues/2563>`_)
* Contributors: Christophe Bedard

28.3.0 (2024-06-17)
-------------------
* Add 'mimick' label to tests which use Mimick (`#2516 <https://github.com/ros2/rclcpp/issues/2516>`_)
* Contributors: Scott K Logan

28.2.0 (2024-04-26)
-------------------

28.1.0 (2024-04-16)
-------------------
* Remove references to index.ros.org. (`#2504 <https://github.com/ros2/rclcpp/issues/2504>`_)
* Contributors: Chris Lalancette

28.0.1 (2024-04-16)
-------------------
* Callback after cancel (`#2281 <https://github.com/ros2/rclcpp/issues/2281>`_)
  * feat(Client): Added function to stop callbacks of a goal handle
  This function allows us to drop the handle in a locked context.
  If we do not do this within a lock, there will be a race condition between
  the deletion of the shared_ptr of the handle and the result / feedback
  callbacks.
  * fix: make Client goal handle recursive
  This fixes deadlocks due to release of goal handles in callbacks etc.
  * fix(ActionGoalClient): Fixed memory leak for nominal case
  This fixes a memory leak due to a self reference in the ClientGoalHandle.
  Note, this fix will only work, if the ClientGoalHandle ever receives
  a result callback.
  * doc: Updated documentation of rclcpp_action::Client::async_send_goal
  * docs: Made the async_send_goal documentation more explicit
  Co-authored-by: Janosch Machowinski <J.Machowinski@cellumation.com>
* Remake of "fix: Fixed race condition in action server between is_ready and take" (`#2495 <https://github.com/ros2/rclcpp/issues/2495>`_)
  Some background information: is_ready, take_data and execute data
  may be called from different threads in any order. The code in the old
  state expected them to be called in series, without interruption.
  This lead to multiple race conditions, as the state of the pimpl objects
  was altered by the three functions in a non thread safe way.
  Co-authored-by: Janosch Machowinski <j.machowinski@nospam.org>
* update rclcpp::Waitable API to use references and const (`#2467 <https://github.com/ros2/rclcpp/issues/2467>`_)
* Contributors: William Woodall, jmachowinski

28.0.0 (2024-03-28)
-------------------
* Do not generate the exception when action service response timeout. (`#2464 <https://github.com/ros2/rclcpp/issues/2464>`_)
  * Do not generate the exception when action service response timeout.
  * address review comment.
  ---------
* Modify rclcpp_action::GoalUUID hashing algorithm (`#2441 <https://github.com/ros2/rclcpp/issues/2441>`_)
  * Add unit tests for hashing rclcpp_action::GoalUUID's
  * Use the FNV-1a hash algorithm for Goal UUID
* Various cleanups to deal with uncrustify 0.78. (`#2439 <https://github.com/ros2/rclcpp/issues/2439>`_)
  These should also work with uncrustify 0.72.
* Update quality declaration documents (`#2427 <https://github.com/ros2/rclcpp/issues/2427>`_)
* Contributors: Chris Lalancette, Christophe Bedard, Tomoya Fujita, mauropasse

27.0.0 (2024-02-07)
-------------------

26.0.0 (2024-01-24)
-------------------

25.0.0 (2023-12-26)
-------------------
* Switch to target_link_libraries. (`#2374 <https://github.com/ros2/rclcpp/issues/2374>`_)
* Contributors: Chris Lalancette

24.0.0 (2023-11-06)
-------------------

23.2.0 (2023-10-09)
-------------------

23.1.0 (2023-10-04)
-------------------

23.0.0 (2023-09-08)
-------------------
* Update API docs links in package READMEs (`#2302 <https://github.com/ros2/rclcpp/issues/2302>`_)
* fix(ClientGoalHandle): Made mutex recursive to prevent deadlocks (`#2267 <https://github.com/ros2/rclcpp/issues/2267>`_)
* Contributors: Christophe Bedard, jmachowinski

22.2.0 (2023-09-07)
-------------------
* Correct the position of a comment. (`#2290 <https://github.com/ros2/rclcpp/issues/2290>`_)
* Fix a typo in a comment. (`#2283 <https://github.com/ros2/rclcpp/issues/2283>`_)
* doc fix: call `canceled` only after goal state is in canceling. (`#2266 <https://github.com/ros2/rclcpp/issues/2266>`_)
* Contributors: Chris Lalancette, Jiaqi Li, Tomoya Fujita

22.1.0 (2023-08-21)
-------------------

22.0.0 (2023-07-11)
-------------------

21.3.0 (2023-06-12)
-------------------

21.2.0 (2023-06-07)
-------------------

21.1.1 (2023-05-11)
-------------------

21.1.0 (2023-04-27)
-------------------

21.0.0 (2023-04-18)
-------------------

20.0.0 (2023-04-13)
-------------------
* extract the result response before the callback is issued. (`#2132 <https://github.com/ros2/rclcpp/issues/2132>`_)
* Update all rclcpp packages to C++17. (`#2121 <https://github.com/ros2/rclcpp/issues/2121>`_)
* Fix the GoalUUID to_string representation (`#1999 <https://github.com/ros2/rclcpp/issues/1999>`_)
* Contributors: Chris Lalancette, Nathan Wiebe Neufeldt, Tomoya Fujita

19.3.0 (2023-03-01)
-------------------

19.2.0 (2023-02-24)
-------------------

19.1.0 (2023-02-14)
-------------------

19.0.0 (2023-01-30)
-------------------

18.0.0 (2022-12-29)
-------------------
* Explicitly set callback type (`#2059 <https://github.com/ros2/rclcpp/issues/2059>`_)
* Update maintainers (`#2043 <https://github.com/ros2/rclcpp/issues/2043>`_)
* Contributors: Audrow Nash, mauropasse

17.1.0 (2022-11-02)
-------------------
* Do not clear entities callbacks on destruction (`#2002 <https://github.com/ros2/rclcpp/issues/2002>`_)
* Contributors: mauropasse

17.0.0 (2022-09-13)
-------------------
* Revert "Introduce executors new spin_for method, replace spin_until_future_complete with spin_until_complete. (`#1821 <https://github.com/ros2/rclcpp/issues/1821>`_) (`#1874 <https://github.com/ros2/rclcpp/issues/1874>`_)" (`#1956 <https://github.com/ros2/rclcpp/issues/1956>`_)
* Introduce executors new spin_for method, replace spin_until_future_complete with spin_until_complete. (`#1821 <https://github.com/ros2/rclcpp/issues/1821>`_) (`#1874 <https://github.com/ros2/rclcpp/issues/1874>`_)
* Contributors: Hubert Liberacki, William Woodall

16.2.0 (2022-05-03)
-------------------

16.1.0 (2022-04-29)
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
