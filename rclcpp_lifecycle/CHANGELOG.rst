^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


16.0.5 (2023-07-17)
-------------------
* Fix thread safety in LifecycleNode::get_current_state() for Humble (`#2183 <https://github.com/ros2/rclcpp/issues/2183>`_)
  * add initially-failing test case
  * apply changes to LifecycleNodeInterfaceImpl from `#1756 <https://github.com/ros2/rclcpp/issues/1756>`_
  * add static member to State for managing state_handle\_ access
  * allow parallel read access in MutexMap
* Contributors: Joseph Schornak

16.0.4 (2023-04-25)
-------------------

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

15.3.0 (2022-03-30)
-------------------

15.2.0 (2022-03-24)
-------------------
* Fix rosdoc2 issues (`#1897 <https://github.com/ros2/rclcpp/issues/1897>`_)
* Contributors: Chris Lalancette

15.1.0 (2022-03-01)
-------------------
* Install headers to include/${PROJECT_NAME} (`#1888 <https://github.com/ros2/rclcpp/issues/1888>`_)
* LifecycleNode::on_deactivate deactivate all managed entities. (`#1885 <https://github.com/ros2/rclcpp/issues/1885>`_)
* Contributors: Shane Loretz, Tomoya Fujita

15.0.0 (2022-01-14)
-------------------
* Automatically transition lifecycle entities when node transitions (`#1863 <https://github.com/ros2/rclcpp/issues/1863>`_)
* Contributors: Ivan Santiago Paunovic

14.1.0 (2022-01-05)
-------------------

14.0.0 (2021-12-17)
-------------------
* Remove author by request (`#1818 <https://github.com/ros2/rclcpp/issues/1818>`_)
* Update maintainers (`#1817 <https://github.com/ros2/rclcpp/issues/1817>`_)
* Suppress clang dead-store warnings in the benchmarks. (`#1802 <https://github.com/ros2/rclcpp/issues/1802>`_)
* Contributors: Chris Lalancette, Jacob Perron

13.1.0 (2021-10-18)
-------------------
* Update forward declarations of `rcl_lifecycle` types (`#1788 <https://github.com/ros2/rclcpp/issues/1788>`_)
* Deprecate the `void shared_ptr<MessageT>` subscription callback signatures (`#1713 <https://github.com/ros2/rclcpp/issues/1713>`_)
* Contributors: Abrar Rahman Protyasha, Michel Hidalgo

13.0.0 (2021-08-23)
-------------------
* Update client API to be able to remove pending requests. (`#1734 <https://github.com/ros2/rclcpp/issues/1734>`_)
* Change log level for lifecycle_publisher. (`#1715 <https://github.com/ros2/rclcpp/issues/1715>`_)
* Fix: RCLCPP_PUBLIC -> RCLCPP_LIFECYCLE_PUBLIC (`#1732 <https://github.com/ros2/rclcpp/issues/1732>`_)
* Use rcpputils/scope_exit.hpp and remove rclcpp/scope_exit.hpp (`#1727 <https://github.com/ros2/rclcpp/issues/1727>`_)
* Contributors: Alberto Soragna, Christophe Bedard, Ivan Santiago Paunovic, Shane Loretz

12.0.0 (2021-07-26)
-------------------
* Remove unsafe get_callback_groups API.
  Callers should change to using for_each_callback_group(), or
  store the callback groups they need internally.
* Add in callback_groups_for_each.
  The main reason to add this method in is to make accesses to the
  callback_groups\_ vector thread-safe.  By having a
  callback_groups_for_each that accepts a std::function, we can
  just have the callers give us the callback they are interested
  in, and we can take care of the locking.
  The rest of this fairly large PR is cleaning up all of the places
  that use get_callback_groups() to instead use
  callback_groups_for_each().
* Contributors: Chris Lalancette

11.2.0 (2021-07-21)
-------------------

11.1.0 (2021-07-13)
-------------------

11.0.0 (2021-05-18)
-------------------
* Fix destruction order in lifecycle benchmark (`#1675 <https://github.com/ros2/rclcpp/issues/1675>`_)
* Contributors: Scott K Logan

10.0.0 (2021-05-11)
-------------------
* [rclcpp] Type Adaptation feature (`#1557 <https://github.com/ros2/rclcpp/issues/1557>`_)
* Contributors: Audrow Nash, William Woodall

9.0.2 (2021-04-14)
------------------

9.0.1 (2021-04-12)
------------------

9.0.0 (2021-04-06)
------------------
* Add generic publisher and generic subscription for serialized messages (`#1452 <https://github.com/ros2/rclcpp/issues/1452>`_)
* updating quality declaration links (re: `ros2/docs.ros2.org#52 <https://github.com/ros2/docs.ros2.org/issues/52>`_) (`#1615 <https://github.com/ros2/rclcpp/issues/1615>`_)
* Contributors: Nikolai Morin, shonigmann

8.2.0 (2021-03-31)
------------------
* Fix flaky lifecycle node tests (`#1606 <https://github.com/ros2/rclcpp/issues/1606>`_)
* Clock subscription callback group spins in its own thread (`#1556 <https://github.com/ros2/rclcpp/issues/1556>`_)
* Delete debug messages (`#1602 <https://github.com/ros2/rclcpp/issues/1602>`_)
* add automatically_add_executor_with_node option (`#1594 <https://github.com/ros2/rclcpp/issues/1594>`_)
* Contributors: BriceRenaudeau, Ivan Santiago Paunovic, Jacob Perron, anaelle-sw

8.1.0 (2021-03-25)
------------------

8.0.0 (2021-03-23)
------------------
* make rcl_lifecyle_com_interface optional in lifecycle nodes (`#1507 <https://github.com/ros2/rclcpp/issues/1507>`_)
* Contributors: Karsten Knese

7.0.1 (2021-03-22)
------------------

7.0.0 (2021-03-18)
------------------
* Add support for rmw_connextdds (`#1574 <https://github.com/ros2/rclcpp/issues/1574>`_)
* Fix SEGV caused by order of destruction of Node sub-interfaces (`#1469 <https://github.com/ros2/rclcpp/issues/1469>`_)
* Enforce static parameter types (`#1522 <https://github.com/ros2/rclcpp/issues/1522>`_)
* Contributors: Andrea Sorbini, Colin MacKenzie, Ivan Santiago Paunovic

6.3.1 (2021-02-08)
------------------

6.3.0 (2021-01-25)
------------------

6.2.0 (2021-01-08)
------------------

6.1.0 (2020-12-10)
------------------
* add LifecycleNode::get_transition_graph to match services. (`#1472 <https://github.com/ros2/rclcpp/issues/1472>`_)
* Update QDs to QL 1 (`#1477 <https://github.com/ros2/rclcpp/issues/1477>`_)
* Benchmark lifecycle features (`#1462 <https://github.com/ros2/rclcpp/issues/1462>`_)
* Contributors: Stephen Brawner, brawner, tomoya

6.0.0 (2020-11-18)
------------------
* Reserve vector capacities and use emplace_back for constructing vectors (`#1464 <https://github.com/ros2/rclcpp/issues/1464>`_)
* [rclcpp_lifecycle] Change uint8_t iterator variables to size_t (`#1461 <https://github.com/ros2/rclcpp/issues/1461>`_)
* Bump rclcpp packages to Quality Level 2 (`#1445 <https://github.com/ros2/rclcpp/issues/1445>`_)
* Contributors: Louise Poubel, brawner

5.1.0 (2020-11-02)
------------------
* Increase test timeouts of slow running tests with rmw_connext_cpp (`#1400 <https://github.com/ros2/rclcpp/issues/1400>`_)
* Update maintainers (`#1384 <https://github.com/ros2/rclcpp/issues/1384>`_)
* Add clock qos to node options (`#1375 <https://github.com/ros2/rclcpp/issues/1375>`_)
* Contributors: Ivan Santiago Paunovic, brawner

5.0.0 (2020-09-18)
------------------
* Increase test coverage of rclcpp_lifecycle to 96% (`#1298 <https://github.com/ros2/rclcpp/issues/1298>`_)
* Log error instead of throwing exception in Transition and State reset(), mark no except (`#1297 <https://github.com/ros2/rclcpp/issues/1297>`_)
* Remove unused private function (rclcpp::Node and rclcpp_lifecycle::Node) (`#1294 <https://github.com/ros2/rclcpp/issues/1294>`_)
* Remove rmw-dependent unit-test checks (`#1293 <https://github.com/ros2/rclcpp/issues/1293>`_)
* Added missing tests for rclcpp lifecycle (`#1240 <https://github.com/ros2/rclcpp/issues/1240>`_)
* Warn about unused result of add_on_set_parameters_callback (`#1238 <https://github.com/ros2/rclcpp/issues/1238>`_)
* Contributors: Alejandro Hernández Cordero, Jacob Perron, Stephen Brawner

4.0.0 (2020-07-09)
------------------
* Remove deprecated set_on_parameters_set_callback function (`#1199 <https://github.com/ros2/rclcpp/issues/1199>`_)
* Bump to QD to level 3 and fixed links (`#1158 <https://github.com/ros2/rclcpp/issues/1158>`_)
* Fix race in test_lifecycle_service_client (`#1204 <https://github.com/ros2/rclcpp/issues/1204>`_)
* Contributors: Alejandro Hernández Cordero, Claire Wang, Dirk Thomas

3.0.0 (2020-06-18)
------------------
* Fix doxygen warnings (`#1163 <https://github.com/ros2/rclcpp/issues/1163>`_)
* Contributors: Alejandro Hernández Cordero

2.0.0 (2020-06-01)
------------------
* Added missing virtual destructors. (`#1149 <https://github.com/ros2/rclcpp/issues/1149>`_)
* Add Security Vulnerability Policy pointing to REP-2006. (`#1130 <https://github.com/ros2/rclcpp/issues/1130>`_)
* Fixed ``test_lifecycle_node.cpp:check_parameters`` (`#1136 <https://github.com/ros2/rclcpp/issues/1136>`_)
* Contributors: Chris Lalancette, Ivan Santiago Paunovic

1.1.0 (2020-05-26)
------------------
* Deprecate set_on_parameters_set_callback (`#1123 <https://github.com/ros2/rclcpp/issues/1123>`_)
* Add missing parameter callback functions to lifecycle node (`#1134 <https://github.com/ros2/rclcpp/issues/1134>`_)
* Expose get_service_names_and_types_by_node from rcl in rclcpp (`#1131 <https://github.com/ros2/rclcpp/issues/1131>`_)
* Improve documentation (`#1106 <https://github.com/ros2/rclcpp/issues/1106>`_)
* Fixed rep links and added more details to dependencies in quality declaration (`#1116 <https://github.com/ros2/rclcpp/issues/1116>`_)
* Update quality declaration to reflect version 1.0 (`#1115 <https://github.com/ros2/rclcpp/issues/1115>`_)
* Contributors: Alejandro Hernández Cordero, Claire Wang, Dirk Thomas, Stephen Brawner

1.0.0 (2020-05-12)
------------------
* Avoid callback_group deprecation (`#1108 <https://github.com/ros2/rclcpp/issues/1108>`_)
* Contributors: Karsten Knese

0.9.1 (2020-05-08)
------------------
* Added rclcpp lifecycle Doxyfile (`#1089 <https://github.com/ros2/rclcpp/issues/1089>`_)
* Added Quality declaration: rclcpp, rclpp_action, rclcpp_components andrclcpp_lifecycle (`#1100 <https://github.com/ros2/rclcpp/issues/1100>`_)
* Increasing test coverage of rclcpp_lifecycle (`#1045 <https://github.com/ros2/rclcpp/issues/1045>`_)
* Contributors: Alejandro Hernández Cordero, brawner

0.9.0 (2020-04-29)
------------------
* Export targets in addition to include directories / libraries (`#1096 <https://github.com/ros2/rclcpp/issues/1096>`_)
* Deprecate redundant namespaces (`#1083 <https://github.com/ros2/rclcpp/issues/1083>`_)
* Integrate topic statistics (`#1072 <https://github.com/ros2/rclcpp/issues/1072>`_)
* Reflect changes in rclcpp API (`#1079 <https://github.com/ros2/rclcpp/issues/1079>`_)
* Fix unknown macro errors reported by cppcheck 1.90 (`#1000 <https://github.com/ros2/rclcpp/issues/1000>`_)
* Rremoved rmw_implementation from package.xml (`#991 <https://github.com/ros2/rclcpp/issues/991>`_)
* Implement functions to get publisher and subcription informations like QoS policies from topic name (`#960 <https://github.com/ros2/rclcpp/issues/960>`_)
* Create node clock calls const (`#922 <https://github.com/ros2/rclcpp/issues/922>`_)
* Type conversions fixes (`#901 <https://github.com/ros2/rclcpp/issues/901>`_)
* Contributors: Alejandro Hernández Cordero, Barry Xu, Devin Bonnie, Dirk Thomas, Jacob Perron, Monika Idzik, Prajakta Gokhale, Steven Macenski, William Woodall

0.8.3 (2019-11-19)
------------------

0.8.2 (2019-11-18)
------------------

0.8.1 (2019-10-23)
------------------
* New Intra-Process Communication (`#778 <https://github.com/ros2/rclcpp/issues/778>`_)
* Contributors: Alberto Soragna

0.8.0 (2019-09-26)
------------------
* clean up publisher and subscription creation logic (`#867 <https://github.com/ros2/rclcpp/issues/867>`_)
* reset error message before setting a new one, embed the original one (`#854 <https://github.com/ros2/rclcpp/issues/854>`_)
* remove features and related code which were deprecated in dashing (`#852 <https://github.com/ros2/rclcpp/issues/852>`_)
* Fix typo in deprecated warning. (`#848 <https://github.com/ros2/rclcpp/issues/848>`_)
* Add line break after first open paren in multiline function call (`#785 <https://github.com/ros2/rclcpp/issues/785>`_)
* Fixe error messages not printing to terminal (`#777 <https://github.com/ros2/rclcpp/issues/777>`_)
* Add default value to options in LifecycleNode construnctor. Update API documentation. (`#775 <https://github.com/ros2/rclcpp/issues/775>`_)
* Contributors: Dan Rose, Dirk Thomas, Esteve Fernandez, Luca Della Vedova, William Woodall, Yathartha Tuladhar

0.7.5 (2019-05-30)
------------------

0.7.4 (2019-05-29)
------------------
* Rename parameter options (`#745 <https://github.com/ros2/rclcpp/issues/745>`_)
* Contributors: William Woodall

0.7.3 (2019-05-20)
------------------
* Added missing template functionality to lifecycle_node. (`#707 <https://github.com/ros2/rclcpp/issues/707>`_)
* Contributors: Michael Jeronimo

0.7.2 (2019-05-08)
------------------
* Added new way to specify QoS settings for publishers and subscriptions. (`#713 <https://github.com/ros2/rclcpp/issues/713>`_)
* Deprecated ``shared_ptr`` and raw pointer versions of ``Publisher<T>::publish()``. (`#709 <https://github.com/ros2/rclcpp/issues/709>`_)
* Implemented API to set callbacks for liveliness and deadline QoS events for publishers and subscriptions. (`#695 <https://github.com/ros2/rclcpp/issues/695>`_)
* Changed the ``IntraProcessManager`` to be capable of storing ``shared_ptr<const T>`` in addition to ``unique_ptr<T>``. (`#690 <https://github.com/ros2/rclcpp/issues/690>`_)
* Contributors: M. M, William Woodall, ivanpauno

0.7.1 (2019-04-26)
------------------
* Added read only parameters. (`#495 <https://github.com/ros2/rclcpp/issues/495>`_)
* Contributors: Shane Loretz, William Woodall

0.7.0 (2019-04-14)
------------------
* Fixed linter errors in rclcpp_lifecycle. (`#672 <https://github.com/ros2/rclcpp/issues/672>`_)
* Added parameter-related templates to LifecycleNode. (`#645 <https://github.com/ros2/rclcpp/issues/645>`_)
* Fixed use_sim_time issue on LifeCycleNode. (`#651 <https://github.com/ros2/rclcpp/issues/651>`_)
* Updated to use ament_target_dependencies where possible. (`#659 <https://github.com/ros2/rclcpp/issues/659>`_)
* Fixed hard-coded duration type representation so int64_t isn't assumed. (`#648 <https://github.com/ros2/rclcpp/issues/648>`_)
* Added a method to the LifecycleNode class to get the logging interface. (`#652 <https://github.com/ros2/rclcpp/issues/652>`_)
* Set Parameter Event Publisher settings `#591 <https://github.com/ros2/rclcpp/issues/591>`_ (`#614 <https://github.com/ros2/rclcpp/issues/614>`_)
* Replaced node constructor arguments with NodeOptions. (`#622 <https://github.com/ros2/rclcpp/issues/622>`_)
* Removed dependency on rclpy. (`#626 <https://github.com/ros2/rclcpp/issues/626>`_)
* Contributors: Emerson Knapp, Karsten Knese, Michael Carroll, Michael Jeronimo, Vinnam Kim, William Woodall, ivanpauno, rarvolt

0.6.2 (2018-12-13)
------------------

0.6.1 (2018-12-07)
------------------
* Added node path and time stamp to parameter event message (`#584 <https://github.com/ros2/rclcpp/issues/584>`_)
* Refactored init to allow for non-global init (`#587 <https://github.com/ros2/rclcpp/issues/587>`_)
* Add class Waitable (`#589 <https://github.com/ros2/rclcpp/issues/589>`_)
* Contributors: Dirk Thomas, Jacob Perron, William Woodall, bpwilcox

0.6.0 (2018-11-19)
------------------
* Updated to use new error handling API from rcutils (`#577 <https://github.com/ros2/rclcpp/issues/577>`_)
* Deleted TRANSITION_SHUTDOWN (`#576 <https://github.com/ros2/rclcpp/issues/576>`_)
* Added a warning when publishing if publisher is not active (`#574 <https://github.com/ros2/rclcpp/issues/574>`_)
* Added SMART_PTRS_DEF to LifecyclePublisher (`#569 <https://github.com/ros2/rclcpp/issues/569>`_)
* Added service for transition graph (`#555 <https://github.com/ros2/rclcpp/issues/555>`_)
* Added semicolons to all RCLCPP and RCUTILS macros. (`#565 <https://github.com/ros2/rclcpp/issues/565>`_)
* Fixed and improved documentation  (`#546 <https://github.com/ros2/rclcpp/issues/546>`_)
* Removed unneeded dependency on std_msgs (`#513 <https://github.com/ros2/rclcpp/issues/513>`_)
* Removed use of uninitialized CMake var (`#511 <https://github.com/ros2/rclcpp/issues/511>`_)
* Added get_node_names API from node. (`#508 <https://github.com/ros2/rclcpp/issues/508>`_)
* Fixed rosidl dependencies (`#507 <https://github.com/ros2/rclcpp/issues/507>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Francisco Martín Rico, Karsten Knese, Mikael Arguedas, Sriram Raghunathan, William Woodall, cho3

0.5.0 (2018-06-25)
------------------
* Added functions that allow you to publish serialized messages and received serialized messages in your subscription callback. (`#388 <https://github.com/ros2/rclcpp/issues/388>`_)
* Added ability to initialize parameter values in a node with an argument to the Node constructor. (`#486 <https://github.com/ros2/rclcpp/issues/486>`_)
* Nodes now autostart the ROS parameter services which let you get, set, and list parameters in a node. (`#478 <https://github.com/ros2/rclcpp/issues/478>`_)
* Fixed a bug that occurred when mixing ``std::shared_ptr`` and ``std::bind``. (`#470 <https://github.com/ros2/rclcpp/issues/470>`_)
* Added ability to pass command line arguments to the Node constructor. (`#461 <https://github.com/ros2/rclcpp/issues/461>`_)
* Changed library export order for static linking. (`#446 <https://github.com/ros2/rclcpp/issues/446>`_)
* Now depends on ``ament_cmake_ros``. (`#444 <https://github.com/ros2/rclcpp/issues/444>`_)
* Updaed code to use logging macros rather than ``fprintf()``. (`#439 <https://github.com/ros2/rclcpp/issues/439>`_)
* Contributors: Dirk Thomas, Guillaume Autran, Karsten Knese, Michael Carroll, Mikael Arguedas, Shane Loretz, dhood
