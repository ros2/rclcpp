^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2018-11-19)
------------------
* Updated to use new error handling API from rcutils (`#577 <https://github.com/ros2/rclcpp/issues/577>`_)
* Added a warning when publishing if publisher is not active (`#574 <https://github.com/ros2/rclcpp/issues/574>`_)
* Added logging macro signature that accepts std::string (`#573 <https://github.com/ros2/rclcpp/issues/573>`_)
* Added virtual destructors to classes with virtual functions. (`#566 <https://github.com/ros2/rclcpp/issues/566>`_)
* Added semicolons to all RCLCPP and RCUTILS macros. (`#565 <https://github.com/ros2/rclcpp/issues/565>`_)
* Removed std::binary_function usage (`#561 <https://github.com/ros2/rclcpp/issues/561>`_)
* Updated to avoid auto-activating ROS time if clock topic is being published (`#559 <https://github.com/ros2/rclcpp/issues/559>`_)
* Fixed cpplint on xenial (`#556 <https://github.com/ros2/rclcpp/issues/556>`_)
* Added get_parameter_or_set_default. (`#551 <https://github.com/ros2/rclcpp/issues/551>`_)
* Added max_duration to spin_some() (`#558 <https://github.com/ros2/rclcpp/issues/558>`_)
* Updated to output rcl error message when yaml parsing fails (`#557 <https://github.com/ros2/rclcpp/issues/557>`_)
* Updated to make sure timer is fini'd before clock (`#553 <https://github.com/ros2/rclcpp/issues/553>`_)
* Get node names and namespaces (`#545 <https://github.com/ros2/rclcpp/issues/545>`_)
* Fixed and improved documentation  (`#546 <https://github.com/ros2/rclcpp/issues/546>`_)
* Updated to use rcl_clock_t jump callbacks (`#543 <https://github.com/ros2/rclcpp/issues/543>`_)
* Updated to use rcl consolidated wait set functions (`#540 <https://github.com/ros2/rclcpp/issues/540>`_)
* Addeed TIME_MAX and DURATION_MAX functions (`#538 <https://github.com/ros2/rclcpp/issues/538>`_)
* Updated to publish shared_ptr of rcl_serialized_message (`#541 <https://github.com/ros2/rclcpp/issues/541>`_)
* Added Time::is_zero and Duration::seconds (`#536 <https://github.com/ros2/rclcpp/issues/536>`_)
* Changed to log an error message instead of throwing exception in destructor (`#535 <https://github.com/ros2/rclcpp/issues/535>`_)
* Updated to relax tolerance of now test because timing affected by OS scheduling (`#533 <https://github.com/ros2/rclcpp/issues/533>`_)
* Removed incorrect exception on sec < 0 (`#527 <https://github.com/ros2/rclcpp/issues/527>`_)
* Added rclcpp::Time::seconds() (`#526 <https://github.com/ros2/rclcpp/issues/526>`_)
* Updated Timer API to construct TimerBase/GenericTimer with Clock (`#523 <https://github.com/ros2/rclcpp/issues/523>`_)
* Added rclcpp::is_initialized() (`#522 <https://github.com/ros2/rclcpp/issues/522>`_)
* Added support for jump handlers with only pre- or post-jump callback (`#517 <https://github.com/ros2/rclcpp/issues/517>`_)
* Removed use of uninitialized CMake var (`#512 <https://github.com/ros2/rclcpp/issues/512>`_)
* Updated for Uncrustify 0.67 (`#510 <https://github.com/ros2/rclcpp/issues/510>`_)
* Added get_node_names API from node. (`#508 <https://github.com/ros2/rclcpp/issues/508>`_)
* Contributors: Anis Ladram, Chris Lalancette, Dirk Thomas, Francisco Martín Rico, Karsten Knese, Michael Carroll, Mikael Arguedas, Sagnik Basu, Shane Loretz, Sriram Raghunathan, William Woodall, chapulina, dhood

0.5.0 (2018-06-25)
------------------
* Fixed a bug in the multi-threaded executor which could cause it to take a timer (potentially other types of wait-able items) more than once to be worked one. (`#383 <https://github.com/ros2/rclcpp/issues/383>`_)
  * Specifically this could result in a timer getting called more often that it should when using the multi-threaded executor.
* Added functions that allow you to publish serialized messages and received serialized messages in your subscription callback. (`#388 <https://github.com/ros2/rclcpp/issues/388>`_)
* Changed code to always get the Service name from ``rcl`` to ensure the remapped name is returned. (`#498 <https://github.com/ros2/rclcpp/issues/498>`_)
* Added previously missing ``set_parameters_atomically()`` method to the Service client interface. (`#494 <https://github.com/ros2/rclcpp/issues/494>`_)
* Added ability to initialize parameter values in a Node via a YAML file passed on the command line. (`#488 <https://github.com/ros2/rclcpp/issues/488>`_)
* Fixed the ROS parameter interface which got parameters that aren't set. (`#493 <https://github.com/ros2/rclcpp/issues/493>`_)
* Added ability to initialize parameter values in a node with an argument to the Node constructor. (`#486 <https://github.com/ros2/rclcpp/issues/486>`_)
* Added a ``Subscription`` tests which uses ``std::bind`` to a class member callback. (`#480 <https://github.com/ros2/rclcpp/issues/480>`_)
* Refactored the ``ParameterVariant`` class into the ``Parameter`` and ``ParameterValue`` classes. (`#481 <https://github.com/ros2/rclcpp/issues/481>`_)
* Relaxed template matching rules for ``std::bind`` and ``GNU C++ >= 7.1``. (`#484 <https://github.com/ros2/rclcpp/issues/484>`_)
* Changed to use the new ``rosgraph_msgs/Clock`` message type for the ``/clock`` topic. (`#474 <https://github.com/ros2/rclcpp/issues/474>`_)
* Fixed a flaky ROS time test due to not spinning before getting the time. (`#483 <https://github.com/ros2/rclcpp/issues/483>`_)
* Nodes now autostart the ROS parameter services which let you get, set, and list parameters in a node. (`#478 <https://github.com/ros2/rclcpp/issues/478>`_)
* Added support for arrays in Parameters. (`#443 <https://github.com/ros2/rclcpp/issues/443>`_)
* Changed how executors use ``AnyExecutable`` objects so that they are a reference instead of a shared pointer, in order to avoid memory allocation in the "common case". (`#463 <https://github.com/ros2/rclcpp/issues/463>`_)
* Added ability to pass command line arguments to the Node constructor. (`#461 <https://github.com/ros2/rclcpp/issues/461>`_)
* Added an argument to specify the number of threads a multithreaded executor should create. (`#442 <https://github.com/ros2/rclcpp/issues/442>`_)
* Changed library export order for static linking. (`#446 <https://github.com/ros2/rclcpp/issues/446>`_)
* Fixed some typos in the time unit tests. (`#453 <https://github.com/ros2/rclcpp/issues/453>`_)
  Obviously it mean RCL_SYSTEM_TIME but not RCL_ROS_TIME in some test cases
  * Signed-off-by: jwang <jing.j.wang@intel.com>
* Added the scale operation to ``rclcpp::Duration``.
  * Signed-off-by: jwang <jing.j.wang@intel.com>
* Changed API of the log location parameter to be ``const``. (`#451 <https://github.com/ros2/rclcpp/issues/451>`_)
* Changed how the subscriber, client, service, and timer handles are stored to resolve shutdown order issues. (`#431 <https://github.com/ros2/rclcpp/issues/431>`_ and `#448 <https://github.com/ros2/rclcpp/issues/448>`_)
* Updated to get the node's logger name from ``rcl``. (`#433 <https://github.com/ros2/rclcpp/issues/433>`_)
* Now depends on ``ament_cmake_ros``. (`#444 <https://github.com/ros2/rclcpp/issues/444>`_)
* Updaed code to use logging macros rather than ``fprintf()``. (`#439 <https://github.com/ros2/rclcpp/issues/439>`_)
* Fixed a bug that was using an invalid iterator when erasing items using an iterator in a loop. (`#436 <https://github.com/ros2/rclcpp/issues/436>`_)
* Changed code to support move of ``rcutils_time_point_value_t`` type from ``uint64_t`` to ``int64_t``. (`#429 <https://github.com/ros2/rclcpp/issues/429>`_)
* Renamed parameter byte type to ``byte_values`` from ``bytes_value``. (`#428 <https://github.com/ros2/rclcpp/issues/428>`_)
* Changed executor code to clear the wait set before resizing and waiting. (`#427 <https://github.com/ros2/rclcpp/issues/427>`_)
* Fixed a potential dereference of nullptr in the topic name validation error string. (`#405 <https://github.com/ros2/rclcpp/issues/405>`_)
  * Signed-off-by: Ethan Gao <ethan.gao@linux.intel.com>
* Changed to use ``rcl_count_publishers()`` like API's rather than the lower level ``rmw_count_publishers()`` API. (`#425 <https://github.com/ros2/rclcpp/issues/425>`_)
  * Signed-off-by: Sriram Raghunathan <rsriram7@visteon.com>
* Fix potential segmentation fault due to ``get_topic_name()`` or ``rcl_service_get_service_name()`` returning nullptr and that not being checked before access in ``rclcpp``. (`#426 <https://github.com/ros2/rclcpp/issues/426>`_)
  * Signed-off-by: Ethan Gao <ethan.gao@linux.intel.com>
* Contributors: Denise Eng, Dirk Thomas, Ernesto Corbellini, Esteve Fernandez, Ethan Gao, Guillaume Autran, Karsten Knese, Matthew, Michael Carroll, Mikael Arguedas, Shane Loretz, Sriram Raghunathan, Tom Moore, William Woodall, dhood, jwang, jwang11, serge-nikulin
