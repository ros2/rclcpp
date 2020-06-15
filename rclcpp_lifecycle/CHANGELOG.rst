^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


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
