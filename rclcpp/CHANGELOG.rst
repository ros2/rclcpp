^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
