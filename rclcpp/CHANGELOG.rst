^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.16 (2021-05-21)
-------------------
* Fix documented example in create_publisher. (`#1558 <https://github.com/ros2/rclcpp/issues/1558>`_) (`#1560 <https://github.com/ros2/rclcpp/issues/1560>`_)
* Fix NodeOptions copy constructor. (`#1376 <https://github.com/ros2/rclcpp/issues/1376>`_) (`#1466 <https://github.com/ros2/rclcpp/issues/1466>`_)
* Contributors: Ivan Santiago Paunovic, Jacob Perron

0.7.15 (2020-11-24)
-------------------
* Fix implementation of NodeOptions::use_global_arguments(). (`#1176 <https://github.com/ros2/rclcpp/issues/1176>`_) (`#1206 <https://github.com/ros2/rclcpp/issues/1206>`_)
* Fix conversion of negative durations to messages. (`#1188 <https://github.com/ros2/rclcpp/issues/1188>`_) (`#1207 <https://github.com/ros2/rclcpp/issues/1207>`_)
* Type conversions fixes. (`#901 <https://github.com/ros2/rclcpp/issues/901>`_) (`#1209 <https://github.com/ros2/rclcpp/issues/1209>`_)
* Add operator!= for duration (`#1236 <https://github.com/ros2/rclcpp/issues/1236>`_) (`#1277 <https://github.com/ros2/rclcpp/issues/1277>`_)
* Contributors: Ivan Santiago Paunovic, Michel Hidalgo, Monika Idzik

0.7.14 (2020-07-10)
-------------------
* Fixed doxygen warnings. (`#1208 <https://github.com/ros2/rclcpp/issues/1208>`_)
* Check if context is valid when looping in spin_some. (`#1167 <https://github.com/ros2/rclcpp/issues/1167>`_)
* Fix spin_until_future_complete: check spinning value. (`#1023 <https://github.com/ros2/rclcpp/issues/1023>`_)
* Fix exception on rcl_clock_init. (`#1195 <https://github.com/ros2/rclcpp/issues/1195>`_)
* Fix lock-order-inversion (potential deadlock). (`#1138 <https://github.com/ros2/rclcpp/issues/1138>`_)
* Contributors: Alejandro Hernández Cordero, DongheeYe, Ivan Santiago Paunovic, tomoya

0.7.13 (2020-03-12)
-------------------
* Don't specify calling convention in std::_Binder template. (`#1015 <https://github.com/ros2/rclcpp/issues/1015>`_)
* Contributors: Jacob Perron, Sean Kelly

0.7.12 (2019-12-05)
-------------------

0.7.11 (2019-10-11)
-------------------
* Fix `get_node_*_interface` functions taking a pointer (`#870 <https://github.com/ros2/rclcpp/pull/870>`_).
* Fix hang with timers in `MultiThreadedExecutor` (`#869 <https://github.com/ros2/rclcpp/pull/869>`_).
* Contributors: Todd Malsbary, ivanpauno

0.7.10 (2019-09-23)
-------------------

0.7.9 (2019-09-20)
------------------
* add mutex in add/remove_node and wait_for_work to protect concurrent use/change of memory_strategy\_ (`#837 <https://github.com/ros2/rclcpp/issues/837>`_) (`#857 <https://github.com/ros2/rclcpp/issues/857>`_)
* Contributors: Zachary Michaels

0.7.8 (2019-09-06)
------------------

0.7.7 (2019-07-31)
------------------
* Enabled the creation of a parameter YAML file which is applied to each node. (`#805 <https://github.com/ros2/rclcpp/issues/805>`_)
* Fixed a signed/unsigned integer comparison compiler warning. (`#804 <https://github.com/ros2/rclcpp/issues/804>`_)
* Changed the QoS profile used when subscribing to parameter events to match the publishing side, i.e. ``rmw_qos_profile_parameter_events``. (`#798 <https://github.com/ros2/rclcpp/issues/798>`_)
* Changed the logic in TimeSource to ignore use_sim_time parameter events coming from other nodes. (`#803 <https://github.com/ros2/rclcpp/issues/803>`_)
* Added missing default values in the NodeParametersInterface. (`#794 <https://github.com/ros2/rclcpp/issues/794>`_)
* Added support for const member callback functions. (`#763 <https://github.com/ros2/rclcpp/issues/763>`_)
* Contributors: Esteve Fernandez, Gonzo, Karsten Knese, Michel Hidalgo, Scott K Logan

0.7.6 (2019-06-12)
------------------
* Ignore parameters overrides in set parameter methods when allowing undeclared parameters (`#756 <https://github.com/ros2/rclcpp/issues/756>`_)
* Add rclcpp::create_timer() (`#757 <https://github.com/ros2/rclcpp/issues/757>`_)
* checking origin of intra-process msg before taking them (`#753 <https://github.com/ros2/rclcpp/issues/753>`_)
* Contributors: Alberto Soragna, Shane Loretz, ivanpauno

0.7.5 (2019-05-30)
------------------
* Avoid 'Intra process message no longer being stored when trying to handle it' warning (`#749 <https://github.com/ros2/rclcpp/issues/749>`_)
* Contributors: ivanpauno

0.7.4 (2019-05-29)
------------------
* Rename parameter options (`#745 <https://github.com/ros2/rclcpp/issues/745>`_)
* Bionic use of strerror_r (`#742 <https://github.com/ros2/rclcpp/issues/742>`_)
* Enforce parameter ranges (`#735 <https://github.com/ros2/rclcpp/issues/735>`_)
* removed not used parameter client (`#740 <https://github.com/ros2/rclcpp/issues/740>`_)
* ensure removal of guard conditions of expired nodes from memory strategy (`#741 <https://github.com/ros2/rclcpp/issues/741>`_)
* Fix typo in log warning message (`#737 <https://github.com/ros2/rclcpp/issues/737>`_)
* Throw nice errors when creating a publisher with intraprocess communication and incompatible qos policy (`#729 <https://github.com/ros2/rclcpp/issues/729>`_)
* Contributors: Alberto Soragna, Dirk Thomas, Jacob Perron, William Woodall, ivanpauno, roderick-koehle

0.7.3 (2019-05-20)
------------------
* Fixed misspelling, volitile -> volatile (`#724 <https://github.com/ros2/rclcpp/issues/724>`_), and then fixed that since it is a C++ keyword to be ``durability_volatile`` (`#725 <https://github.com/ros2/rclcpp/issues/725>`_)
* Fixed a clang warning (`#723 <https://github.com/ros2/rclcpp/issues/723>`_)
* Added ``on_parameter_event`` static method to the ``AsyncParametersClient`` (`#688 <https://github.com/ros2/rclcpp/issues/688>`_)
* Added a guard against ``ParameterNotDeclaredException`` throwing from within the parameter service callbacks. (`#718 <https://github.com/ros2/rclcpp/issues/718>`_)
* Added missing template functionality to lifecycle_node. (`#707 <https://github.com/ros2/rclcpp/issues/707>`_)
* Fixed heap-use-after-free and memory leaks reported from ``test_node.cpp`` (`#719 <https://github.com/ros2/rclcpp/issues/719>`_)
* Contributors: Alberto Soragna, Dirk Thomas, Emerson Knapp, Jacob Perron, Michael Jeronimo, Prajakta Gokhale

0.7.2 (2019-05-08)
------------------
* Added new way to specify QoS settings for publishers and subscriptions. (`#713 <https://github.com/ros2/rclcpp/issues/713>`_)
  * The new way requires that you specify a history depth when creating a publisher or subscription.
  * In the past it was possible to create one without specifying any history depth, but these signatures have been deprecated.
* Deprecated ``shared_ptr`` and raw pointer versions of ``Publisher<T>::publish()``. (`#709 <https://github.com/ros2/rclcpp/issues/709>`_)
* Implemented API to set callbacks for liveliness and deadline QoS events for publishers and subscriptions. (`#695 <https://github.com/ros2/rclcpp/issues/695>`_)
* Fixed a segmentation fault when publishing a parameter event when they ought to be disabled. (`#714 <https://github.com/ros2/rclcpp/issues/714>`_)
* Changes required for upcoming pre-allocation API. (`#711 <https://github.com/ros2/rclcpp/issues/711>`_)
* Changed ``Node::get_node_names()`` to return the full node names rather than just the base name. (`#698 <https://github.com/ros2/rclcpp/issues/698>`_)
* Remove logic made redundant by the `ros2/rcl#255 <https://github.com/ros2/rcl/issues/255>`_ pull request. (`#712 <https://github.com/ros2/rclcpp/issues/712>`_)
* Various improvements for ``rclcpp::Clock``. (`#696 <https://github.com/ros2/rclcpp/issues/696>`_)
  * Fixed uninitialized bool in ``clock.cpp``.
  * Fixed up includes of ``clock.hpp/cpp``.
  * Added documentation for exceptions to ``clock.hpp``.
  * Adjusted function signature of getters of ``clock.hpp/cpp``.
  * Removed raw pointers to ``Clock::create_jump_callback``.
  * Removed unnecessary ``rclcpp`` namespace reference from ``clock.cpp``.
  * Changed exception to ``bad_alloc`` on ``JumpHandler`` allocation failure.
  * Fixed missing ``nullptr`` check in ``Clock::on_time_jump``.
  * Added ``JumpHandler::callback`` types.
  * Added warning for lifetime of Clock and JumpHandler
* Fixed bug left over from the `pull request #495 <https://github.com/ros2/rclcpp/pull/495>`_. (`#708 <https://github.com/ros2/rclcpp/issues/708>`_)
* Changed the ``IntraProcessManager`` to be capable of storing ``shared_ptr<const T>`` in addition to ``unique_ptr<T>``. (`#690 <https://github.com/ros2/rclcpp/issues/690>`_)
* Contributors: Alberto Soragna, Dima Dorezyuk, M. M, Michael Carroll, Michael Jeronimo, Tully Foote, William Woodall, ivanpauno, jhdcs

0.7.1 (2019-04-26)
------------------
* Added read only parameters. (`#495 <https://github.com/ros2/rclcpp/issues/495>`_)
* Fixed a concurrency problem in the multithreaded executor. (`#703 <https://github.com/ros2/rclcpp/issues/703>`_)
* Fixup utilities. (`#692 <https://github.com/ros2/rclcpp/issues/692>`_)
* Added method to read timer cancellation. (`#697 <https://github.com/ros2/rclcpp/issues/697>`_)
* Added Exception Generator function for implementing "from_rcl_error". (`#678 <https://github.com/ros2/rclcpp/issues/678>`_)
* Updated initialization of rmw_qos_profile_t struct instances. (`#691 <https://github.com/ros2/rclcpp/issues/691>`_)
* Removed the const value from the logger before comparison. (`#680 <https://github.com/ros2/rclcpp/issues/680>`_)
* Contributors: Devin Bonnie, Dima Dorezyuk, Guillaume Autran, M. M, Shane Loretz, Víctor Mayoral Vilches, William Woodall, jhdcs

0.7.0 (2019-04-14)
------------------
* Added Options-struct interfaces for creating publishers/subscribers (pre-QoS, standalone). (`#673 <https://github.com/ros2/rclcpp/issues/673>`_)
* Replaced strncpy with memcpy. (`#684 <https://github.com/ros2/rclcpp/issues/684>`_)
* Replaced const char * with a std::array<char, TOPIC_NAME_LENGTH> as the key of IPM IDTopicMap. (`#671 <https://github.com/ros2/rclcpp/issues/671>`_)
* Refactored SignalHandler logger to avoid race during destruction. (`#682 <https://github.com/ros2/rclcpp/issues/682>`_)
* Introduce rclcpp_components to implement composition. (`#665 <https://github.com/ros2/rclcpp/issues/665>`_)
* Added QoS policy check when configuring intraprocess, skip interprocess publish when possible. (`#674 <https://github.com/ros2/rclcpp/issues/674>`_)
* Updated to use do { .. } while(0) around content of logging macros. (`#681 <https://github.com/ros2/rclcpp/issues/681>`_)
* Added function to get publisher's actual QoS settings. (`#667 <https://github.com/ros2/rclcpp/issues/667>`_)
* Updated to avoid race that triggers timer too often. (`#621 <https://github.com/ros2/rclcpp/issues/621>`_)
* Exposed get_fully_qualified_name in NodeBase API. (`#662 <https://github.com/ros2/rclcpp/issues/662>`_)
* Updated to use ament_target_dependencies where possible. (`#659 <https://github.com/ros2/rclcpp/issues/659>`_)
* Fixed wait for service memory leak bug. (`#656 <https://github.com/ros2/rclcpp/issues/656>`_)
* Fixed test_time_source test. (`#639 <https://github.com/ros2/rclcpp/issues/639>`_)
* Fixed hard-coded duration type representation so int64_t isn't assumed. (`#648 <https://github.com/ros2/rclcpp/issues/648>`_)
* Fixed cppcheck warning. (`#646 <https://github.com/ros2/rclcpp/issues/646>`_)
* Added count matching api and intra-process subscriber count. (`#628 <https://github.com/ros2/rclcpp/issues/628>`_)
* Added Sub Node alternative. (`#581 <https://github.com/ros2/rclcpp/issues/581>`_)
* Replaced 'auto' with 'const auto &'. (`#630 <https://github.com/ros2/rclcpp/issues/630>`_)
* Set Parameter Event Publisher settings. `#591 <https://github.com/ros2/rclcpp/issues/591>`_ (`#614 <https://github.com/ros2/rclcpp/issues/614>`_)
* Replaced node constructor arguments with NodeOptions. (`#622 <https://github.com/ros2/rclcpp/issues/622>`_)
* Updated to pass context to wait set (`#617 <https://github.com/ros2/rclcpp/issues/617>`_)
* Added API to get parameters in a map. (`#575 <https://github.com/ros2/rclcpp/issues/575>`_)
* Updated Bind usage since it is is no longer in std::__1. (`#618 <https://github.com/ros2/rclcpp/issues/618>`_)
* Fixed errors from uncrustify v0.68. (`#613 <https://github.com/ros2/rclcpp/issues/613>`_)
* Added new constructors for SyncParameterClient. (`#612 <https://github.com/ros2/rclcpp/issues/612>`_)
* Contributors: Alberto Soragna, Chris Lalancette, Dirk Thomas, Emerson Knapp, Francisco Martín Rico, Jacob Perron, Marko Durkovic, Michael Carroll, Peter Baughman, Shane Loretz, Wei Liu, William Woodall, Yutaka Kondo, ivanpauno, kuzai, rarvolt

0.6.2 (2018-12-13)
------------------
* Updated to use signal safe synchronization with platform specific semaphores (`#607 <https://github.com/ros2/rclcpp/issues/607>`_)
* Resolved startup race condition for sim time (`#608 <https://github.com/ros2/rclcpp/issues/608>`_)
  Resolves `#595 <https://github.com/ros2/rclcpp/issues/595>`_
* Contributors: Tully Foote, William Woodall

0.6.1 (2018-12-07)
------------------
* Added wait_for_action_server() for action clients (`#598 <https://github.com/ros2/rclcpp/issues/598>`_)
* Added node path and time stamp to parameter event message (`#584 <https://github.com/ros2/rclcpp/issues/584>`_)
* Updated to allow removing a waitable (`#597 <https://github.com/ros2/rclcpp/issues/597>`_)
* Refactored init to allow for non-global init (`#587 <https://github.com/ros2/rclcpp/issues/587>`_)
* Fixed wrong use of constructor and hanging test (`#596 <https://github.com/ros2/rclcpp/issues/596>`_)
* Added class Waitable (`#589 <https://github.com/ros2/rclcpp/issues/589>`_)
* Updated rcl_wait_set_add\_* calls (`#586 <https://github.com/ros2/rclcpp/issues/586>`_)
* Contributors: Dirk Thomas, Jacob Perron, Shane Loretz, William Woodall, bpwilcox

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
