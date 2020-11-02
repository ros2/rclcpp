^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Deprecate Duration(rcl_duration_value_t) in favor of static Duration::from_nanoseconds(rcl_duration_value_t) (`#1432 <https://github.com/ros2/rclcpp/issues/1432>`_)
* Avoid parsing arguments twice in `rclcpp::init_and_remove_ros_arguments` (`#1415 <https://github.com/ros2/rclcpp/issues/1415>`_)
* Add service and client benchmarks (`#1425 <https://github.com/ros2/rclcpp/issues/1425>`_)
* Set CMakeLists to only use default rmw for benchmarks (`#1427 <https://github.com/ros2/rclcpp/issues/1427>`_)
* Update tracetools' QL in rclcpp's QD (`#1428 <https://github.com/ros2/rclcpp/issues/1428>`_)
* Add missing locking to the rclcpp_action::ServerBase. (`#1421 <https://github.com/ros2/rclcpp/issues/1421>`_)
* Initial benchmark tests for rclcpp::init/shutdown create/destroy node (`#1411 <https://github.com/ros2/rclcpp/issues/1411>`_)
* Refactor test CMakeLists in prep for benchmarks (`#1422 <https://github.com/ros2/rclcpp/issues/1422>`_)
* Add methods in topic and service interface to resolve a name (`#1410 <https://github.com/ros2/rclcpp/issues/1410>`_)
* Update deprecated gtest macros (`#1370 <https://github.com/ros2/rclcpp/issues/1370>`_)
* Clear members for StaticExecutorEntitiesCollector to avoid shared_ptr dependency (`#1303 <https://github.com/ros2/rclcpp/issues/1303>`_)
* Increase test timeouts of slow running tests with rmw_connext_cpp (`#1400 <https://github.com/ros2/rclcpp/issues/1400>`_)
* Avoid self dependency that not destoryed (`#1301 <https://github.com/ros2/rclcpp/issues/1301>`_)
* Update maintainers (`#1384 <https://github.com/ros2/rclcpp/issues/1384>`_)
* Add clock qos to node options (`#1375 <https://github.com/ros2/rclcpp/issues/1375>`_)
* Fix NodeOptions copy constructor (`#1376 <https://github.com/ros2/rclcpp/issues/1376>`_)
* Make sure to clean the external client/service handle. (`#1296 <https://github.com/ros2/rclcpp/issues/1296>`_)
* Increase coverage of WaitSetTemplate (`#1368 <https://github.com/ros2/rclcpp/issues/1368>`_)
* Increase coverage of guard_condition.cpp to 100% (`#1369 <https://github.com/ros2/rclcpp/issues/1369>`_)
* Add coverage statement (`#1367 <https://github.com/ros2/rclcpp/issues/1367>`_)
* Tests for LoanedMessage with mocked loaned message publisher (`#1366 <https://github.com/ros2/rclcpp/issues/1366>`_)
* Add unit tests for qos and qos_event files (`#1352 <https://github.com/ros2/rclcpp/issues/1352>`_)
* Finish coverage of publisher API (`#1365 <https://github.com/ros2/rclcpp/issues/1365>`_)
* Finish API coverage on executors. (`#1364 <https://github.com/ros2/rclcpp/issues/1364>`_)
* Add test for ParameterService (`#1355 <https://github.com/ros2/rclcpp/issues/1355>`_)
* Add time API coverage tests (`#1347 <https://github.com/ros2/rclcpp/issues/1347>`_)
* Add timer coverage tests (`#1363 <https://github.com/ros2/rclcpp/issues/1363>`_)
* Add in additional tests for parameter_client.cpp coverage.
* Minor fixes to the parameter_service.cpp file.
* reset rcl_context shared_ptr after calling rcl_init sucessfully (`#1357 <https://github.com/ros2/rclcpp/issues/1357>`_)
* Improved test publisher - zero qos history depth value exception (`#1360 <https://github.com/ros2/rclcpp/issues/1360>`_)
* Covered resolve_use_intra_process (`#1359 <https://github.com/ros2/rclcpp/issues/1359>`_)
* Improve test_subscription_options (`#1358 <https://github.com/ros2/rclcpp/issues/1358>`_)
* Add in more tests for init_options coverage. (`#1353 <https://github.com/ros2/rclcpp/issues/1353>`_)
* Test the remaining node public API (`#1342 <https://github.com/ros2/rclcpp/issues/1342>`_)
* Complete coverage of Parameter and ParameterValue API (`#1344 <https://github.com/ros2/rclcpp/issues/1344>`_)
* Add in more tests for the utilities. (`#1349 <https://github.com/ros2/rclcpp/issues/1349>`_)
* Add in two more tests for expand_topic_or_service_name. (`#1350 <https://github.com/ros2/rclcpp/issues/1350>`_)
* Add tests for node_options API (`#1343 <https://github.com/ros2/rclcpp/issues/1343>`_)
* Add in more coverage for expand_topic_or_service_name. (`#1346 <https://github.com/ros2/rclcpp/issues/1346>`_)
* Test exception in spin_until_future_complete. (`#1345 <https://github.com/ros2/rclcpp/issues/1345>`_)
* Add coverage tests graph_listener (`#1330 <https://github.com/ros2/rclcpp/issues/1330>`_)
* Add in unit tests for the Executor class.
* Allow mimick patching of methods with up to 9 arguments.
* Improve the error messages in the Executor class.
* Add coverage for client API (`#1329 <https://github.com/ros2/rclcpp/issues/1329>`_)
* Increase service coverage (`#1332 <https://github.com/ros2/rclcpp/issues/1332>`_)
* Make more of the static entity collector API private.
* Const-ify more of the static executor.
* Add more tests for the static single threaded executor.
* Many more tests for the static_executor_entities_collector.
* Get one more line of code coverage in memory_strategy.cpp
* Bugfix when adding callback group.
* Fix typos in comments.
* Remove deprecated executor::FutureReturnCode APIs. (`#1327 <https://github.com/ros2/rclcpp/issues/1327>`_)
* Increase coverage of publisher/subscription API (`#1325 <https://github.com/ros2/rclcpp/issues/1325>`_)
* Not finalize guard condition while destructing SubscriptionIntraProcess (`#1307 <https://github.com/ros2/rclcpp/issues/1307>`_)
* Expose qos setting for /rosout (`#1247 <https://github.com/ros2/rclcpp/issues/1247>`_)
* Add coverage for missing API (except executors) (`#1326 <https://github.com/ros2/rclcpp/issues/1326>`_)
* Include topic name in QoS mismatch warning messages (`#1286 <https://github.com/ros2/rclcpp/issues/1286>`_)
* Add coverage tests context functions (`#1321 <https://github.com/ros2/rclcpp/issues/1321>`_)
* Increase coverage of node_interfaces, including with mocking rcl errors (`#1322 <https://github.com/ros2/rclcpp/issues/1322>`_)
* Contributors: Ada-King, Alejandro Hernández Cordero, Audrow Nash, Barry Xu, Chen Lihui, Chris Lalancette, Christophe Bedard, Ivan Santiago Paunovic, Jorge Perez, Morgan Quigley, brawner

5.0.0 (2020-09-18)
------------------
* Make node_graph::count_graph_users() const (`#1320 <https://github.com/ros2/rclcpp/issues/1320>`_)
* Add coverage for wait_set_policies (`#1316 <https://github.com/ros2/rclcpp/issues/1316>`_)
* Only exchange intra_process waitable if nonnull (`#1317 <https://github.com/ros2/rclcpp/issues/1317>`_)
* Check waitable for nullptr during constructor (`#1315 <https://github.com/ros2/rclcpp/issues/1315>`_)
* Call vector.erase with end iterator overload (`#1314 <https://github.com/ros2/rclcpp/issues/1314>`_)
* Use best effort, keep last, history depth 1 QoS Profile for '/clock' subscriptions (`#1312 <https://github.com/ros2/rclcpp/issues/1312>`_)
* Add tests type_support module (`#1308 <https://github.com/ros2/rclcpp/issues/1308>`_)
* Replace std_msgs with test_msgs in executors test (`#1310 <https://github.com/ros2/rclcpp/issues/1310>`_)
* Add set_level for rclcpp::Logger (`#1284 <https://github.com/ros2/rclcpp/issues/1284>`_)
* Remove unused private function (rclcpp::Node and rclcpp_lifecycle::Node) (`#1294 <https://github.com/ros2/rclcpp/issues/1294>`_)
* Adding tests basic getters (`#1291 <https://github.com/ros2/rclcpp/issues/1291>`_)
* Adding callback groups in executor (`#1218 <https://github.com/ros2/rclcpp/issues/1218>`_)
* Refactor Subscription Topic Statistics Tests (`#1281 <https://github.com/ros2/rclcpp/issues/1281>`_)
* Add operator!= for duration (`#1236 <https://github.com/ros2/rclcpp/issues/1236>`_)
* Fix clock thread issue (`#1266 <https://github.com/ros2/rclcpp/issues/1266>`_) (`#1267 <https://github.com/ros2/rclcpp/issues/1267>`_)
* Fix topic stats test, wait for more messages, only check the ones with samples (`#1274 <https://github.com/ros2/rclcpp/issues/1274>`_)
* Add get_domain_id method to rclcpp::Context (`#1271 <https://github.com/ros2/rclcpp/issues/1271>`_)
* Fixes for unit tests that fail under cyclonedds (`#1270 <https://github.com/ros2/rclcpp/issues/1270>`_)
* initialize_logging\_ should be copied (`#1272 <https://github.com/ros2/rclcpp/issues/1272>`_)
* Use static_cast instead of C-style cast for instrumentation (`#1263 <https://github.com/ros2/rclcpp/issues/1263>`_)
* Make parameter clients use template constructors (`#1249 <https://github.com/ros2/rclcpp/issues/1249>`_)
* Ability to configure domain_id via InitOptions. (`#1165 <https://github.com/ros2/rclcpp/issues/1165>`_)
* Simplify and fix allocator memory strategy unit test for connext (`#1252 <https://github.com/ros2/rclcpp/issues/1252>`_)
* Use global namespace for parameter events subscription topic (`#1257 <https://github.com/ros2/rclcpp/issues/1257>`_)
* Increase timeouts for connext for long tests (`#1253 <https://github.com/ros2/rclcpp/issues/1253>`_)
* Adjust test_static_executor_entities_collector for rmw_connext_cpp (`#1251 <https://github.com/ros2/rclcpp/issues/1251>`_)
* Fix failing test with Connext since it doesn't wait for discovery (`#1246 <https://github.com/ros2/rclcpp/issues/1246>`_)
* Fix node graph test with Connext and CycloneDDS returning actual data (`#1245 <https://github.com/ros2/rclcpp/issues/1245>`_)
* Warn about unused result of add_on_set_parameters_callback (`#1238 <https://github.com/ros2/rclcpp/issues/1238>`_)
* Unittests for memory strategy files, except allocator_memory_strategy (`#1189 <https://github.com/ros2/rclcpp/issues/1189>`_)
* EXPECT_THROW_EQ and ASSERT_THROW_EQ macros for unittests (`#1232 <https://github.com/ros2/rclcpp/issues/1232>`_)
* Add unit test for static_executor_entities_collector (`#1221 <https://github.com/ros2/rclcpp/issues/1221>`_)
* Parameterize test executors for all executor types (`#1222 <https://github.com/ros2/rclcpp/issues/1222>`_)
* Unit tests for allocator_memory_strategy.cpp part 2 (`#1198 <https://github.com/ros2/rclcpp/issues/1198>`_)
* Unit tests for allocator_memory_strategy.hpp (`#1197 <https://github.com/ros2/rclcpp/issues/1197>`_)
* Derive and throw exception in spin_some spin_all for StaticSingleThreadedExecutor (`#1220 <https://github.com/ros2/rclcpp/issues/1220>`_)
* Make ring buffer thread-safe (`#1213 <https://github.com/ros2/rclcpp/issues/1213>`_)
* Add missing RCLCPP_PUBLIC to ~StaticExecutorEntitiesCollector (`#1227 <https://github.com/ros2/rclcpp/issues/1227>`_)
* Document graph functions don't apply remap rules (`#1225 <https://github.com/ros2/rclcpp/issues/1225>`_)
* Remove recreation of entities_collector (`#1217 <https://github.com/ros2/rclcpp/issues/1217>`_)
* Contributors: Audrow Nash, Chen Lihui, Christophe Bedard, Daisuke Sato, Devin Bonnie, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Jannik Abbenseth, Jorge Perez, Pedro Pena, Shane Loretz, Stephen Brawner, Tomoya Fujita

4.0.0 (2020-07-09)
------------------
* Fix rclcpp::NodeOptions::operator= (`#1211 <https://github.com/ros2/rclcpp/issues/1211>`_)
* Link against thread library where necessary (`#1210 <https://github.com/ros2/rclcpp/issues/1210>`_)
* Unit tests for node interfaces (`#1202 <https://github.com/ros2/rclcpp/issues/1202>`_)
* Remove usage of domain id in node options (`#1205 <https://github.com/ros2/rclcpp/issues/1205>`_)
* Remove deprecated set_on_parameters_set_callback function (`#1199 <https://github.com/ros2/rclcpp/issues/1199>`_)
* Fix conversion of negative durations to messages (`#1188 <https://github.com/ros2/rclcpp/issues/1188>`_)
* Fix implementation of NodeOptions::use_global_arguments() (`#1176 <https://github.com/ros2/rclcpp/issues/1176>`_)
* Bump to QD to level 3 and fixed links (`#1158 <https://github.com/ros2/rclcpp/issues/1158>`_)
* Fix pub/sub count API tests (`#1203 <https://github.com/ros2/rclcpp/issues/1203>`_)
* Update tracetools' QL to 2 in rclcpp's QD (`#1187 <https://github.com/ros2/rclcpp/issues/1187>`_)
* Fix exception message on rcl_clock_init (`#1182 <https://github.com/ros2/rclcpp/issues/1182>`_)
* Throw exception if rcl_timer_init fails (`#1179 <https://github.com/ros2/rclcpp/issues/1179>`_)
* Unit tests for some header-only functions/classes (`#1181 <https://github.com/ros2/rclcpp/issues/1181>`_)
* Callback should be perfectly-forwarded (`#1183 <https://github.com/ros2/rclcpp/issues/1183>`_)
* Add unit tests for logging functionality (`#1184 <https://github.com/ros2/rclcpp/issues/1184>`_)
* Add create_publisher include to create_subscription (`#1180 <https://github.com/ros2/rclcpp/issues/1180>`_)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard, Claire Wang, Dirk Thomas, Ivan Santiago Paunovic, Johannes Meyer, Michel Hidalgo, Stephen Brawner, tomoya

3.0.0 (2020-06-18)
------------------
* Check period duration in create_wall_timer (`#1178 <https://github.com/ros2/rclcpp/issues/1178>`_)
* Fix get_node_time_source_interface() docstring (`#988 <https://github.com/ros2/rclcpp/issues/988>`_)
* Add message lost subscription event (`#1164 <https://github.com/ros2/rclcpp/issues/1164>`_)
* Add spin_all method to Executor (`#1156 <https://github.com/ros2/rclcpp/issues/1156>`_)
* Reorganize test directory and split CMakeLists.txt (`#1173 <https://github.com/ros2/rclcpp/issues/1173>`_)
* Check if context is valid when looping in spin_some (`#1167 <https://github.com/ros2/rclcpp/issues/1167>`_)
* Add check for invalid topic statistics publish period (`#1151 <https://github.com/ros2/rclcpp/issues/1151>`_)
* Fix spin_until_future_complete: check spinning value (`#1023 <https://github.com/ros2/rclcpp/issues/1023>`_)
* Fix doxygen warnings (`#1163 <https://github.com/ros2/rclcpp/issues/1163>`_)
* Fix reference to rclcpp in its Quality declaration (`#1161 <https://github.com/ros2/rclcpp/issues/1161>`_)
* Allow spin_until_future_complete to accept any future like object (`#1113 <https://github.com/ros2/rclcpp/issues/1113>`_)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard, Devin Bonnie, Dirk Thomas, DongheeYe, Ivan Santiago Paunovic, Jacob Perron, Sarthak Mittal, brawner, tomoya

2.0.0 (2020-06-01)
------------------
* Added missing virtual destructors. (`#1149 <https://github.com/ros2/rclcpp/issues/1149>`_)
* Fixed a test which was using different types on the same topic. (`#1150 <https://github.com/ros2/rclcpp/issues/1150>`_)
* Made ``test_rate`` more reliable on Windows and improve error output when it fails (`#1146 <https://github.com/ros2/rclcpp/issues/1146>`_)
* Added Security Vulnerability Policy pointing to REP-2006. (`#1130 <https://github.com/ros2/rclcpp/issues/1130>`_)
* Added missing header in ``logging_mutex.cpp``. (`#1145 <https://github.com/ros2/rclcpp/issues/1145>`_)
* Changed the WaitSet API to pass a shared pointer by value instead than by const reference when possible. (`#1141 <https://github.com/ros2/rclcpp/issues/1141>`_)
* Changed ``SubscriptionBase::get_subscription_handle() const`` to return a shared pointer to const value. (`#1140 <https://github.com/ros2/rclcpp/issues/1140>`_)
* Extended the lifetime of ``rcl_publisher_t`` by holding onto the shared pointer in order to avoid a use after free situation. (`#1119 <https://github.com/ros2/rclcpp/issues/1119>`_)
* Improved some docblocks (`#1127 <https://github.com/ros2/rclcpp/issues/1127>`_)
* Fixed a lock-order-inversion (potential deadlock) (`#1135 <https://github.com/ros2/rclcpp/issues/1135>`_)
* Fixed a potential Construction/Destruction order problem between global contexts vector and Context of static lifetime (`#1132 <https://github.com/ros2/rclcpp/issues/1132>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Ivan Santiago Paunovic, Michel Hidalgo, tomoya

1.1.0 (2020-05-26)
------------------
* Deprecate set_on_parameters_set_callback (`#1123 <https://github.com/ros2/rclcpp/issues/1123>`_)
* Expose get_service_names_and_types_by_node from rcl in rclcpp (`#1131 <https://github.com/ros2/rclcpp/issues/1131>`_)
* Fix thread safety issues related to logging (`#1125 <https://github.com/ros2/rclcpp/issues/1125>`_)
* Make sure rmw_publisher_options is initialized in to_rcl_publisher_options (`#1099 <https://github.com/ros2/rclcpp/issues/1099>`_)
* Remove empty lines within method signatures (`#1128 <https://github.com/ros2/rclcpp/issues/1128>`_)
* Add API review March 2020 document (`#1031 <https://github.com/ros2/rclcpp/issues/1031>`_)
* Improve documentation (`#1106 <https://github.com/ros2/rclcpp/issues/1106>`_)
* Make test multi threaded executor more reliable (`#1105 <https://github.com/ros2/rclcpp/issues/1105>`_)
* Fixed rep links and added more details to dependencies in quality declaration (`#1116 <https://github.com/ros2/rclcpp/issues/1116>`_)
* Update quality declarations to reflect version 1.0 (`#1115 <https://github.com/ros2/rclcpp/issues/1115>`_)
* Contributors: Alejandro Hernández Cordero, ChenYing Kuo, Claire Wang, Dirk Thomas, Ivan Santiago Paunovic, William Woodall, Stephen Brawner

1.0.0 (2020-05-12)
------------------
* Remove MANUAL_BY_NODE liveliness API (`#1107 <https://github.com/ros2/rclcpp/issues/1107>`_)
* Use rosidl_default_generators dependency in test (`#1114 <https://github.com/ros2/rclcpp/issues/1114>`_)
* Make sure to include what you use (`#1112 <https://github.com/ros2/rclcpp/issues/1112>`_)
* Mark flaky test with xfail: TestMultiThreadedExecutor (`#1109 <https://github.com/ros2/rclcpp/issues/1109>`_)
* Contributors: Chris Lalancette, Ivan Santiago Paunovic, Karsten Knese, Louise Poubel

0.9.1 (2020-05-08)
------------------
* Fix tests that were not properly torn down (`#1073 <https://github.com/ros2/rclcpp/issues/1073>`_)
* Added docblock in rclcpp (`#1103 <https://github.com/ros2/rclcpp/issues/1103>`_)
* Added Quality declaration: rclcpp, rclpp_action, rclcpp_components andrclcpp_lifecycle (`#1100 <https://github.com/ros2/rclcpp/issues/1100>`_)
* Use RCL_RET_SERVICE_TAKE_FAILED and not RCL_RET_CLIENT_TAKE_FAILED when checking a request take (`#1101 <https://github.com/ros2/rclcpp/issues/1101>`_)
* Update comment about return value in Executor::get_next_ready_executable (`#1085 <https://github.com/ros2/rclcpp/issues/1085>`_)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard, Devin Bonnie, Ivan Santiago Paunovic

0.9.0 (2020-04-29)
------------------
* Serialized message move constructor (`#1097 <https://github.com/ros2/rclcpp/issues/1097>`_)
* Enforce a precedence for wildcard matching in parameter overrides. (`#1094 <https://github.com/ros2/rclcpp/issues/1094>`_)
* Add serialized_message.hpp header (`#1095 <https://github.com/ros2/rclcpp/issues/1095>`_)
* Add received message age metric to topic statistics (`#1080 <https://github.com/ros2/rclcpp/issues/1080>`_)
* Deprecate redundant namespaces (`#1083 <https://github.com/ros2/rclcpp/issues/1083>`_)
* Export targets in addition to include directories / libraries (`#1088 <https://github.com/ros2/rclcpp/issues/1088>`_)
* Ensure logging is initialized just once (`#998 <https://github.com/ros2/rclcpp/issues/998>`_)
* Adapt subscription traits to rclcpp::SerializedMessage (`#1092 <https://github.com/ros2/rclcpp/issues/1092>`_)
* Protect subscriber_statistics_collectors\_ with a mutex (`#1084 <https://github.com/ros2/rclcpp/issues/1084>`_)
* Remove unused test variable (`#1087 <https://github.com/ros2/rclcpp/issues/1087>`_)
* Use serialized message (`#1081 <https://github.com/ros2/rclcpp/issues/1081>`_)
* Integrate topic statistics (`#1072 <https://github.com/ros2/rclcpp/issues/1072>`_)
* Fix rclcpp interface traits test (`#1086 <https://github.com/ros2/rclcpp/issues/1086>`_)
* Generate node interfaces' getters and traits (`#1069 <https://github.com/ros2/rclcpp/issues/1069>`_)
* Use composition for serialized message (`#1082 <https://github.com/ros2/rclcpp/issues/1082>`_)
* Dnae adas/serialized message (`#1075 <https://github.com/ros2/rclcpp/issues/1075>`_)
* Reflect changes in rclcpp API (`#1079 <https://github.com/ros2/rclcpp/issues/1079>`_)
* Fix build regression (`#1078 <https://github.com/ros2/rclcpp/issues/1078>`_)
* Add NodeDefault option for enabling topic statistics (`#1074 <https://github.com/ros2/rclcpp/issues/1074>`_)
* Topic Statistics: Add SubscriptionTopicStatistics class (`#1050 <https://github.com/ros2/rclcpp/issues/1050>`_)
* Add SubscriptionOptions for topic statistics (`#1057 <https://github.com/ros2/rclcpp/issues/1057>`_)
* Remove warning message from failing to register default callback (`#1067 <https://github.com/ros2/rclcpp/issues/1067>`_)
* Create a default warning for qos incompatibility (`#1051 <https://github.com/ros2/rclcpp/issues/1051>`_)
* Add WaitSet class and modify entities to work without executor (`#1047 <https://github.com/ros2/rclcpp/issues/1047>`_)
* Include what you use (`#1059 <https://github.com/ros2/rclcpp/issues/1059>`_)
* Rename rosidl_generator_cpp namespace to rosidl_runtime_cpp (`#1060 <https://github.com/ros2/rclcpp/issues/1060>`_)
* Changed rosidl_generator_c/cpp to rosidl_runtime_c/cpp (`#1014 <https://github.com/ros2/rclcpp/issues/1014>`_)
* Use constexpr for endpoint type name (`#1055 <https://github.com/ros2/rclcpp/issues/1055>`_)
* Add InvalidParameterTypeException (`#1027 <https://github.com/ros2/rclcpp/issues/1027>`_)
* Support for ON_REQUESTED_INCOMPATIBLE_QOS and ON_OFFERED_INCOMPATIBLE_QOS events (`#924 <https://github.com/ros2/rclcpp/issues/924>`_)
* Fixup clang warning (`#1040 <https://github.com/ros2/rclcpp/issues/1040>`_)
* Adding a "static" single threaded executor (`#1034 <https://github.com/ros2/rclcpp/issues/1034>`_)
* Add equality operators for QoS profile (`#1032 <https://github.com/ros2/rclcpp/issues/1032>`_)
* Remove extra vertical whitespace (`#1030 <https://github.com/ros2/rclcpp/issues/1030>`_)
* Switch IntraProcessMessage to test_msgs/Empty (`#1017 <https://github.com/ros2/rclcpp/issues/1017>`_)
* Add new type of exception that may be thrown during creation of publisher/subscription (`#1026 <https://github.com/ros2/rclcpp/issues/1026>`_)
* Don't check lifespan on publisher QoS (`#1002 <https://github.com/ros2/rclcpp/issues/1002>`_)
* Fix get_parameter_tyeps of AsyncPrameterClient results are always empty (`#1019 <https://github.com/ros2/rclcpp/issues/1019>`_)
* Cleanup node interfaces includes (`#1016 <https://github.com/ros2/rclcpp/issues/1016>`_)
* Add ifdefs to remove tracing-related calls if tracing is disabled (`#1001 <https://github.com/ros2/rclcpp/issues/1001>`_)
* Include missing header in node_graph.cpp (`#994 <https://github.com/ros2/rclcpp/issues/994>`_)
* Add missing includes of logging.hpp (`#995 <https://github.com/ros2/rclcpp/issues/995>`_)
* Zero initialize publisher GID in subscription intra process callback (`#1011 <https://github.com/ros2/rclcpp/issues/1011>`_)
* Removed ament_cmake dependency (`#989 <https://github.com/ros2/rclcpp/issues/989>`_)
* Switch to using new rcutils_strerror (`#993 <https://github.com/ros2/rclcpp/issues/993>`_)
* Ensure all rclcpp::Clock accesses are thread-safe
* Use a PIMPL for rclcpp::Clock implementation
* Replace rmw_implementation for rmw dependency in package.xml (`#990 <https://github.com/ros2/rclcpp/issues/990>`_)
* Add missing service callback registration tracepoint (`#986 <https://github.com/ros2/rclcpp/issues/986>`_)
* Rename rmw_topic_endpoint_info_array count to size (`#996 <https://github.com/ros2/rclcpp/issues/996>`_)
* Implement functions to get publisher and subcription informations like QoS policies from topic name (`#960 <https://github.com/ros2/rclcpp/issues/960>`_)
* Code style only: wrap after open parenthesis if not in one line (`#977 <https://github.com/ros2/rclcpp/issues/977>`_)
* Accept taking an rvalue ref future in spin_until_future_complete (`#971 <https://github.com/ros2/rclcpp/issues/971>`_)
* Allow node clock use in logging macros (`#969 <https://github.com/ros2/rclcpp/issues/969>`_) (`#970 <https://github.com/ros2/rclcpp/issues/970>`_)
* Change order of deprecated and visibility attributes (`#968 <https://github.com/ros2/rclcpp/issues/968>`_)
* Deprecated is_initialized() (`#967 <https://github.com/ros2/rclcpp/issues/967>`_)
* Don't specify calling convention in std::_Binder template (`#952 <https://github.com/ros2/rclcpp/issues/952>`_)
* Added missing include to logging.hpp (`#964 <https://github.com/ros2/rclcpp/issues/964>`_)
* Assigning make_shared result to variables in test (`#963 <https://github.com/ros2/rclcpp/issues/963>`_)
* Fix unused parameter warning (`#962 <https://github.com/ros2/rclcpp/issues/962>`_)
* Stop retaining ownership of the rcl context in GraphListener (`#946 <https://github.com/ros2/rclcpp/issues/946>`_)
* Clear sub contexts when starting another init-shutdown cycle (`#947 <https://github.com/ros2/rclcpp/issues/947>`_)
* Avoid possible UB in Clock jump callbacks (`#954 <https://github.com/ros2/rclcpp/issues/954>`_)
* Handle unknown global ROS arguments (`#951 <https://github.com/ros2/rclcpp/issues/951>`_)
* Mark get_clock() as override to fix clang warnings (`#939 <https://github.com/ros2/rclcpp/issues/939>`_)
* Create node clock calls const (try 2) (`#922 <https://github.com/ros2/rclcpp/issues/922>`_)
* Fix asserts on shared_ptr::use_count; expects long, got uint32 (`#936 <https://github.com/ros2/rclcpp/issues/936>`_)
* Use absolute topic name for parameter events (`#929 <https://github.com/ros2/rclcpp/issues/929>`_)
* Add enable_rosout into NodeOptions. (`#900 <https://github.com/ros2/rclcpp/issues/900>`_)
* Removing "virtual", adding "override" keywords (`#897 <https://github.com/ros2/rclcpp/issues/897>`_)
* Use weak_ptr to store context in GraphListener (`#906 <https://github.com/ros2/rclcpp/issues/906>`_)
* Complete published event message when declaring a parameter (`#928 <https://github.com/ros2/rclcpp/issues/928>`_)
* Fix duration.cpp lint error (`#930 <https://github.com/ros2/rclcpp/issues/930>`_)
* Intra-process subscriber should use RMW actual qos. (ros2`#913 <https://github.com/ros2/rclcpp/issues/913>`_) (`#914 <https://github.com/ros2/rclcpp/issues/914>`_)
* Type conversions fixes (`#901 <https://github.com/ros2/rclcpp/issues/901>`_)
* Add override keyword to functions
* Remove unnecessary virtual keywords
* Only check for new work once in spin_some (`#471 <https://github.com/ros2/rclcpp/issues/471>`_) (`#844 <https://github.com/ros2/rclcpp/issues/844>`_)
* Add addition/subtraction assignment operators to Time (`#748 <https://github.com/ros2/rclcpp/issues/748>`_)
* Contributors: Alberto Soragna, Alejandro Hernández Cordero, Barry Xu, Chris Lalancette, Christophe Bedard, Claire Wang, Dan Rose, DensoADAS, Devin Bonnie, Dino Hüllmann, Dirk Thomas, DongheeYe, Emerson Knapp, Ivan Santiago Paunovic, Jacob Perron, Jaison Titus, Karsten Knese, Matt Schickler, Miaofei Mei, Michel Hidalgo, Mikael Arguedas, Monika Idzik, Prajakta Gokhale, Roger Strain, Scott K Logan, Sean Kelly, Stephen Brawner, Steven Macenski, Steven! Ragnarök, Todd Malsbary, Tomoya Fujita, William Woodall, Zachary Michaels

0.8.3 (2019-11-19)
------------------

0.8.2 (2019-11-18)
------------------
* Updated tracing logic to match changes in rclcpp's intra-process system (`#918 <https://github.com/ros2/rclcpp/issues/918>`_)
* Fixed a bug that prevented the ``shutdown_on_sigint`` option to not work correctly (`#850 <https://github.com/ros2/rclcpp/issues/850>`_)
* Added support for STREAM logging macros (`#926 <https://github.com/ros2/rclcpp/issues/926>`_)
* Relaxed multithreaded test constraint (`#907 <https://github.com/ros2/rclcpp/issues/907>`_)
* Contributors: Anas Abou Allaban, Christophe Bedard, Dirk Thomas, alexfneves

0.8.1 (2019-10-23)
------------------
* De-flake tests for rmw_connext (`#899 <https://github.com/ros2/rclcpp/issues/899>`_)
* rename return functions for loaned messages (`#896 <https://github.com/ros2/rclcpp/issues/896>`_)
* Enable throttling logs (`#879 <https://github.com/ros2/rclcpp/issues/879>`_)
* New Intra-Process Communication (`#778 <https://github.com/ros2/rclcpp/issues/778>`_)
* Instrumentation update (`#789 <https://github.com/ros2/rclcpp/issues/789>`_)
* Zero copy api (`#864 <https://github.com/ros2/rclcpp/issues/864>`_)
* Drop rclcpp remove_ros_arguments_null test case. (`#894 <https://github.com/ros2/rclcpp/issues/894>`_)
* add mechanism to pass rmw impl specific payloads during pub/sub creation (`#882 <https://github.com/ros2/rclcpp/issues/882>`_)
* make get_actual_qos return a rclcpp::QoS (`#883 <https://github.com/ros2/rclcpp/issues/883>`_)
* Fix Compiler Warning (`#881 <https://github.com/ros2/rclcpp/issues/881>`_)
* Add callback handler for use_sim_time parameter `#802 <https://github.com/ros2/rclcpp/issues/802>`_ (`#875 <https://github.com/ros2/rclcpp/issues/875>`_)
* Contributors: Alberto Soragna, Brian Marchi, Hunter L. Allen, Ingo Lütkebohle, Karsten Knese, Michael Carroll, Michel Hidalgo, William Woodall

0.8.0 (2019-09-26)
------------------
* clean up publisher and subscription creation logic (`#867 <https://github.com/ros2/rclcpp/issues/867>`_)
* Take parameter overrides provided through the CLI. (`#865 <https://github.com/ros2/rclcpp/issues/865>`_)
* add more context to exception message (`#858 <https://github.com/ros2/rclcpp/issues/858>`_)
* remove features and related code which were deprecated in dashing (`#852 <https://github.com/ros2/rclcpp/issues/852>`_)
* check valid timer handler 1st to reduce the time window for scan. (`#841 <https://github.com/ros2/rclcpp/issues/841>`_)
* Add throwing parameter name if parameter is not set (`#833 <https://github.com/ros2/rclcpp/issues/833>`_)
* Fix typo in deprecated warning. (`#848 <https://github.com/ros2/rclcpp/issues/848>`_)
* Fail on invalid and unknown ROS specific arguments (`#842 <https://github.com/ros2/rclcpp/issues/842>`_)
* Force explicit --ros-args in NodeOptions::arguments(). (`#845 <https://github.com/ros2/rclcpp/issues/845>`_)
* Use of -r/--remap flags where appropriate. (`#834 <https://github.com/ros2/rclcpp/issues/834>`_)
* Fix hang with timers in MultiThreadedExecutor (`#835 <https://github.com/ros2/rclcpp/issues/835>`_) (`#836 <https://github.com/ros2/rclcpp/issues/836>`_)
* add mutex in add/remove_node and wait_for_work to protect concurrent use/change of memory_strategy\_ (`#837 <https://github.com/ros2/rclcpp/issues/837>`_)
* Crash in callback group pointer vector iterator (`#814 <https://github.com/ros2/rclcpp/issues/814>`_)
* Wrap documentation examples in code blocks (`#830 <https://github.com/ros2/rclcpp/issues/830>`_)
* add callback group as member variable and constructor arg (`#811 <https://github.com/ros2/rclcpp/issues/811>`_)
* Fix get_node_interfaces functions taking a pointer (`#821 <https://github.com/ros2/rclcpp/issues/821>`_)
* Delete unnecessary call for get_node_by_group (`#823 <https://github.com/ros2/rclcpp/issues/823>`_)
* Allow passing logger by const ref (`#820 <https://github.com/ros2/rclcpp/issues/820>`_)
* Explain return value of spin_until_future_complete (`#792 <https://github.com/ros2/rclcpp/issues/792>`_)
* Adapt to '--ros-args ... [--]'-based ROS args extraction (`#816 <https://github.com/ros2/rclcpp/issues/816>`_)
* Add line break after first open paren in multiline function call (`#785 <https://github.com/ros2/rclcpp/issues/785>`_)
* remove mock msgs from rclcpp (`#800 <https://github.com/ros2/rclcpp/issues/800>`_)
* Make TimeSource ignore use_sim_time events coming from other nodes. (`#799 <https://github.com/ros2/rclcpp/issues/799>`_)
* Allow registering multiple on_parameters_set_callback (`#772 <https://github.com/ros2/rclcpp/issues/772>`_)
* Add free function for creating service clients (`#788 <https://github.com/ros2/rclcpp/issues/788>`_)
* Include missing rcl headers in use. (`#782 <https://github.com/ros2/rclcpp/issues/782>`_)
* Switch the NodeParameters lock to recursive. (`#781 <https://github.com/ros2/rclcpp/issues/781>`_)
* changed on_parameter_event qos profile to rmw_qos_profile_parameter_events (`#774 <https://github.com/ros2/rclcpp/issues/774>`_)
* Adding a factory method to create a Duration from seconds (`#567 <https://github.com/ros2/rclcpp/issues/567>`_)
* Fix a comparison with a sign mismatch (`#771 <https://github.com/ros2/rclcpp/issues/771>`_)
* delete superfluous spaces (`#770 <https://github.com/ros2/rclcpp/issues/770>`_)
* Use params from node '/\*\*' from parameter YAML file (`#762 <https://github.com/ros2/rclcpp/issues/762>`_)
* Add ignore override argument to declare parameter (`#767 <https://github.com/ros2/rclcpp/issues/767>`_)
* use default parameter descriptor in parameters interface (`#765 <https://github.com/ros2/rclcpp/issues/765>`_)
* Added support for const member functions (`#763 <https://github.com/ros2/rclcpp/issues/763>`_)
* add get_actual_qos() feature to subscriptions (`#754 <https://github.com/ros2/rclcpp/issues/754>`_)
* Ignore parameters overrides in set parameter methods when allowing undeclared parameters (`#756 <https://github.com/ros2/rclcpp/issues/756>`_)
* Add rclcpp::create_timer() (`#757 <https://github.com/ros2/rclcpp/issues/757>`_)
* checking origin of intra-process msg before taking them (`#753 <https://github.com/ros2/rclcpp/issues/753>`_)
* Contributors: Alberto Soragna, Carl Delsey, Chris Lalancette, Dan Rose, Dirk Thomas, Esteve Fernandez, Guillaume Autran, Jacob Perron, Karsten Knese, Luca Della Vedova, M. M, Michel Hidalgo, Scott K Logan, Shane Loretz, Todd Malsbary, William Woodall, bpwilcox, fujitatomoya, ivanpauno

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
