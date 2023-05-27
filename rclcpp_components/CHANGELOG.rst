^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_components
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.3 (2023-05-27)
------------------

2.4.2 (2022-07-25)
------------------

2.4.1 (2022-01-31)
------------------

2.4.0 (2021-09-01)
------------------

2.3.1 (2021-04-14)
------------------
* Update quality declaration links (re: `ros2/docs.ros2.org#52 <https://github.com/ros2/docs.ros2.org/issues/52>`_) (`#1616 <https://github.com/ros2/rclcpp/issues/1616>`_)
* Contributors: Simon Honigmann

2.3.0 (2020-12-09)
------------------
* Update QD to QL 1 (`#1480 <https://github.com/ros2/rclcpp/issues/1480>`_)
* Add benchmarks for components (`#1479 <https://github.com/ros2/rclcpp/issues/1479>`_)
* Contributors: Alejandro Hernández Cordero, Louise Poubel, Scott K Logan, Stephen Brawner

2.2.0 (2020-10-07)
------------------
* Include original exception in ComponentManagerException (`#1157 <https://github.com/ros2/rclcpp/issues/1157>`_) (`#1223 <https://github.com/ros2/rclcpp/issues/1223>`_)
* Contributors: Dereck Wonnacott

2.1.0 (2020-08-03)
------------------

2.0.2 (2020-07-07)
------------------

2.0.1 (2020-06-23)
------------------

2.0.0 (2020-06-01)
------------------
* Added missing virtual destructors (`#1149 <https://github.com/ros2/rclcpp/issues/1149>`_)
* Add Security Vulnerability Policy pointing to REP-2006 (`#1130 <https://github.com/ros2/rclcpp/issues/1130>`_)
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
