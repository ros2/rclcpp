^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_lifecycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.16 (2021-05-21)
-------------------

0.7.15 (2020-11-24)
-------------------
* Type conversions fixes. (`#901 <https://github.com/ros2/rclcpp/issues/901>`_) (`#1209 <https://github.com/ros2/rclcpp/issues/1209>`_)
* Contributors: Michel Hidalgo, Monika Idzik

0.7.14 (2020-07-10)
-------------------
* Fixed doxygen warnings. (`#1208 <https://github.com/ros2/rclcpp/issues/1208>`_)
* Contributors: Alejandro Hernández Cordero

0.7.13 (2020-03-12)
-------------------

0.7.12 (2019-12-05)
-------------------

0.7.11 (2019-10-11)
-------------------

0.7.10 (2019-09-23)
-------------------
* reset error message before setting a new one, embed the original one (`#854 <https://github.com/ros2/rclcpp/issues/854>`_) (`#866 <https://github.com/ros2/rclcpp/issues/866>`_)
* Contributors: Dirk Thomas, Zachary Michaels

0.7.9 (2019-09-20)
------------------

0.7.8 (2019-09-06)
------------------
* Fixed error messages which were not printing to console. (`#777 <https://github.com/ros2/rclcpp/issues/777>`_) (`#847 <https://github.com/ros2/rclcpp/issues/847>`_)
* Contributors: Jacob Perron

0.7.7 (2019-07-31)
------------------
* Added a default value to node options in LifecycleNode constructor to match node constructor. Updated API documentation. (`#801 <https://github.com/ros2/rclcpp/issues/801>`_)
* Contributors: Esteve Fernandez, Dirk Thomas

0.7.6 (2019-06-12)
------------------

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
