^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_components
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

16.0.1 (2022-04-13)
-------------------

16.0.0 (2022-04-08)
-------------------

15.4.0 (2022-04-05)
-------------------

15.3.0 (2022-03-30)
-------------------

15.2.0 (2022-03-24)
-------------------
* Select executor in node registration (`#1898 <https://github.com/ros2/rclcpp/issues/1898>`_)
* Fix rosdoc2 issues in rclcpp (`#1897 <https://github.com/ros2/rclcpp/issues/1897>`_)
* Fix bugprone-exception-escape in node_main.cpp.in (`#1895 <https://github.com/ros2/rclcpp/issues/1895>`_)
* Contributors: Alberto Soragna, Chris Lalancette, Hirokazu Ishida

15.1.0 (2022-03-01)
-------------------
* small improvements to node_main.cpp.in
* Install headers to include/${PROJECT_NAME} (`#1888 <https://github.com/ros2/rclcpp/issues/1888>`_)
* Use spin() in component_manager_isolated.hpp (`#1881 <https://github.com/ros2/rclcpp/issues/1881>`_)
* add use_global_arguments for node options of component nodes (`#1776 <https://github.com/ros2/rclcpp/issues/1776>`_)
* Contributors: Alberto Soragna, Shane Loretz, gezp

15.0.0 (2022-01-14)
-------------------
* Add rclcpp_components::component (`#1855 <https://github.com/ros2/rclcpp/issues/1855>`_)
* Contributors: Shane Loretz

14.1.0 (2022-01-05)
-------------------

14.0.0 (2021-12-17)
-------------------
* Add parameter to configure number of thread (`#1708 <https://github.com/ros2/rclcpp/issues/1708>`_)
* remove RCLCPP_COMPONENTS_PUBLIC in class ComponentManagerIsolated (`#1843 <https://github.com/ros2/rclcpp/issues/1843>`_)
* create component_container_isolated (`#1781 <https://github.com/ros2/rclcpp/issues/1781>`_)
* Remove author by request (`#1818 <https://github.com/ros2/rclcpp/issues/1818>`_)
* Update maintainers (`#1817 <https://github.com/ros2/rclcpp/issues/1817>`_)
* Suppress clang dead-store warnings in the benchmarks. (`#1802 <https://github.com/ros2/rclcpp/issues/1802>`_)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Jacob Perron, gezp

13.1.0 (2021-10-18)
-------------------

13.0.0 (2021-08-23)
-------------------
* Update client API to be able to remove pending requests. (`#1734 <https://github.com/ros2/rclcpp/issues/1734>`_)
* Contributors: Ivan Santiago Paunovic

12.0.0 (2021-07-26)
-------------------

11.2.0 (2021-07-21)
-------------------
* Deprecate method names that use CamelCase in rclcpp_components. (`#1716 <https://github.com/ros2/rclcpp/issues/1716>`_)
* Contributors: Rebecca Butler

11.1.0 (2021-07-13)
-------------------
* Added a hook to generate node options in ComponentManager (`#1702 <https://github.com/ros2/rclcpp/issues/1702>`_)
* Contributors: Rebecca Butler

11.0.0 (2021-05-18)
-------------------

10.0.0 (2021-05-11)
-------------------

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

6.3.1 (2021-02-08)
------------------

6.3.0 (2021-01-25)
------------------

6.2.0 (2021-01-08)
------------------
* Use std compliant non-method std::filesystem::exists function (`#1502 <https://github.com/ros2/rclcpp/issues/1502>`_)
* Fix string literal warnings (`#1442 <https://github.com/ros2/rclcpp/issues/1442>`_)
* Contributors: Audrow Nash, Josh Langsfeld

6.1.0 (2020-12-10)
------------------
* Update QDs to QL 1 (`#1477 <https://github.com/ros2/rclcpp/issues/1477>`_)
* Add benchmarks for components (`#1476 <https://github.com/ros2/rclcpp/issues/1476>`_)
* Contributors: Scott K Logan, Stephen Brawner

6.0.0 (2020-11-18)
------------------
* Bump rclcpp packages to Quality Level 2 (`#1445 <https://github.com/ros2/rclcpp/issues/1445>`_)
* Contributors: Louise Poubel

5.1.0 (2020-11-02)
------------------
* Update maintainers (`#1384 <https://github.com/ros2/rclcpp/issues/1384>`_)
* ComponentManager: switch off parameter services and event publisher (`#1333 <https://github.com/ros2/rclcpp/issues/1333>`_)
* Contributors: Ivan Santiago Paunovic, Martijn Buijs

5.0.0 (2020-09-18)
------------------

4.0.0 (2020-07-09)
------------------
* Bump to QD to level 3 and fixed links (`#1158 <https://github.com/ros2/rclcpp/issues/1158>`_)
* Include original exception in ComponentManagerException (`#1157 <https://github.com/ros2/rclcpp/issues/1157>`_)
* Contributors: Alejandro Hernández Cordero, Martijn Buijs, Tomoya Fujita

3.0.0 (2020-06-18)
------------------

2.0.0 (2020-06-01)
------------------
* Added missing virtual destructors. (`#1149 <https://github.com/ros2/rclcpp/issues/1149>`_)
* Add Security Vulnerability Policy pointing to REP-2006. (`#1130 <https://github.com/ros2/rclcpp/issues/1130>`_)
* Contributors: Chris Lalancette, Ivan Santiago Paunovic

1.1.0 (2020-05-26)
------------------
* Improve documentation (`#1106 <https://github.com/ros2/rclcpp/issues/1106>`_)
* Fixed rep links and added more details to dependencies in quality declaration (`#1116 <https://github.com/ros2/rclcpp/issues/1116>`_)
* Added dockblock to ComponentManager class (`#1102 <https://github.com/ros2/rclcpp/issues/1102>`_)
* Update quality declaration to reflect version 1.0 (`#1115 <https://github.com/ros2/rclcpp/issues/1115>`_)
* Contributors: Alejandro Hernández Cordero, Stephen Brawner

1.0.0 (2020-05-12)
------------------
* Increasing test coverage of rclcpp_components (`#1044 <https://github.com/ros2/rclcpp/issues/1044>`_)
  * Increasing test coverage of rclcpp_components
  Signed-off-by: Stephen Brawner <brawner@gmail.com>
  * PR fixup
  Signed-off-by: Stephen Brawner <brawner@gmail.com>
  * Fixup
  Signed-off-by: Stephen Brawner <brawner@gmail.com>
  * Removing throws test for now
  Signed-off-by: Stephen Brawner <brawner@gmail.com>
* Contributors: brawner

0.9.1 (2020-05-08)
------------------
* Added Quality declaration: rclcpp, rclpp_action, rclcpp_components andrclcpp_lifecycle (`#1100 <https://github.com/ros2/rclcpp/issues/1100>`_)
* Contributors: Alejandro Hernández Cordero

0.9.0 (2020-04-29)
------------------
* Added rclcpp_components Doxyfile (`#1091 <https://github.com/ros2/rclcpp/issues/1091>`_)
* Deprecate redundant namespaces (`#1083 <https://github.com/ros2/rclcpp/issues/1083>`_)
* Export targets in addition to include directories / libraries (`#1088 <https://github.com/ros2/rclcpp/issues/1088>`_)
* Export component manager (`#1070 <https://github.com/ros2/rclcpp/issues/1070>`_)
* Install the component_manager library (`#1068 <https://github.com/ros2/rclcpp/issues/1068>`_)
* Make Component Manager public (`#1065 <https://github.com/ros2/rclcpp/issues/1065>`_)
* Remove absolute path from installed CMake code (`#948 <https://github.com/ros2/rclcpp/issues/948>`_)
* Fix function docblock, check for unparsed arguments (`#945 <https://github.com/ros2/rclcpp/issues/945>`_)
* Contributors: Alejandro Hernández Cordero, DensoADAS, Dirk Thomas, Jacob Perron, Karsten Knese, Michael Carroll, William Woodall

0.8.3 (2019-11-19)
------------------

0.8.2 (2019-11-18)
------------------

0.8.1 (2019-10-23)
------------------
* Enable intra-process comm via LoadNode request. (`#871 <https://github.com/ros2/rclcpp/issues/871>`_)
* Aggregate all component manager API tests. (`#876 <https://github.com/ros2/rclcpp/issues/876>`_)
* Contributors: Michel Hidalgo

0.8.0 (2019-09-26)
------------------
* Force explicit --ros-args in NodeOptions::arguments(). (`#845 <https://github.com/ros2/rclcpp/issues/845>`_)
* Use of -r/--remap flags where appropriate. (`#834 <https://github.com/ros2/rclcpp/issues/834>`_)
* Add line break after first open paren in multiline function call (`#785 <https://github.com/ros2/rclcpp/issues/785>`_)
* fix linter issue (`#795 <https://github.com/ros2/rclcpp/issues/795>`_)
* Remove non-package from ament_target_dependencies() (`#793 <https://github.com/ros2/rclcpp/issues/793>`_)
* fix for multiple nodes not being recognized (`#790 <https://github.com/ros2/rclcpp/issues/790>`_)
* Cmake infrastructure for creating components (`#784 <https://github.com/ros2/rclcpp/issues/784>`_)
* Contributors: Dan Rose, Michel Hidalgo, Shane Loretz, Siddharth Kucheria

0.7.5 (2019-05-30)
------------------

0.7.4 (2019-05-29)
------------------
* Rename parameter options (`#745 <https://github.com/ros2/rclcpp/issues/745>`_)
* don't use global arguments for components loaded into the manager (`#736 <https://github.com/ros2/rclcpp/issues/736>`_)
* Contributors: Dirk Thomas, William Woodall

0.7.3 (2019-05-20)
------------------

0.7.2 (2019-05-08)
------------------
* Updated to support changes to ``Node::get_node_names()``. (`#698 <https://github.com/ros2/rclcpp/issues/698>`_)
* Contributors: jhdcs

0.7.1 (2019-04-26)
------------------

0.7.0 (2019-04-14)
------------------
* Introduce rclcpp_components to implement composition (`#665 <https://github.com/ros2/rclcpp/issues/665>`_)
* Contributors: Michael Carroll

0.6.2 (2018-12-12)
------------------

0.6.1 (2018-12-06)
------------------

0.6.0 (2018-11-19)
------------------

0.5.1 (2018-06-28)
------------------

0.5.0 (2018-06-25)
------------------

0.4.0 (2017-12-08)
------------------
