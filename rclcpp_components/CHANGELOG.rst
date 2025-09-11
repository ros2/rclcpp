^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rclcpp_components
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

29.5.3 (2025-09-11)
-------------------
* fix cmake deprecation (`#2914 <https://github.com/ros2/rclcpp/issues/2914>`_) (`#2915 <https://github.com/ros2/rclcpp/issues/2915>`_)
* Contributors: mergify[bot]

29.5.2 (2025-07-07)
-------------------

29.5.1 (2025-06-23)
-------------------
* Added missing chrono includes (`#2854 <https://github.com/ros2/rclcpp/issues/2854>`_) (`#2855 <https://github.com/ros2/rclcpp/issues/2855>`_)
* Contributors: mergify[bot]

29.5.0 (2025-04-18)
-------------------

29.4.0 (2025-04-04)
-------------------
* Removed trailing whitespace from the codebase. (`#2791 <https://github.com/ros2/rclcpp/issues/2791>`_)
* add NO_UNDEFINED_SYMBOLS to rclcpp_components_register_node cmake macro (`#2746 <https://github.com/ros2/rclcpp/issues/2746>`_) (`#2764 <https://github.com/ros2/rclcpp/issues/2764>`_)
* use maybe_unused attribute for the portability. (`#2758 <https://github.com/ros2/rclcpp/issues/2758>`_)
* ComponentManager should just ignore unknown extra argument in the bas… (`#2723 <https://github.com/ros2/rclcpp/issues/2723>`_)
* Contributors: Leander Stephen D'Souza, Tomoya Fujita, Jonas Otto

29.3.0 (2024-12-20)
-------------------
* Add parsing for rest of obvious boolean extra arguments and throw for unsupported ones (`#2685 <https://github.com/ros2/rclcpp/issues/2685>`_)
* Contributors: rcp1

29.2.0 (2024-11-25)
-------------------

29.1.0 (2024-11-20)
-------------------

29.0.0 (2024-10-03)
-------------------
* Shutdown the context before context's destructor is invoked in tests (`#2633 <https://github.com/ros2/rclcpp/issues/2633>`_)
* Fix typo in rclcpp_components benchmark_components (`#2602 <https://github.com/ros2/rclcpp/issues/2602>`_)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard

28.3.3 (2024-07-29)
-------------------

28.3.2 (2024-07-24)
-------------------
* Updated rcpputils path API (`#2579 <https://github.com/ros2/rclcpp/issues/2579>`_)
* remove deprecated APIs from component_manager.hpp (`#2585 <https://github.com/ros2/rclcpp/issues/2585>`_)
* Contributors: Alberto Soragna, Alejandro Hernández Cordero

28.3.1 (2024-06-25)
-------------------

28.3.0 (2024-06-17)
-------------------

28.2.0 (2024-04-26)
-------------------

28.1.0 (2024-04-16)
-------------------
* Remove references to index.ros.org. (`#2504 <https://github.com/ros2/rclcpp/issues/2504>`_)
* Contributors: Chris Lalancette

28.0.1 (2024-04-16)
-------------------

28.0.0 (2024-03-28)
-------------------
* Add EXECUTOR docs (`#2440 <https://github.com/ros2/rclcpp/issues/2440>`_)
* Update quality declaration documents (`#2427 <https://github.com/ros2/rclcpp/issues/2427>`_)
* crash on no class found (`#2415 <https://github.com/ros2/rclcpp/issues/2415>`_)
  * crash on no class found
  * error on no class found instead of no callback groups
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Contributors: Adam Aposhian, Christophe Bedard, Ruddick Lawrence

27.0.0 (2024-02-07)
-------------------

26.0.0 (2024-01-24)
-------------------

25.0.0 (2023-12-26)
-------------------
* Switch to target_link_libraries. (`#2374 <https://github.com/ros2/rclcpp/issues/2374>`_)
* feat(rclcpp_components): support events executor in node main template (`#2366 <https://github.com/ros2/rclcpp/issues/2366>`_)
* fix(rclcpp_components): increase the service queue sizes in component_container (`#2363 <https://github.com/ros2/rclcpp/issues/2363>`_)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, M. Fatih Cırıt

24.0.0 (2023-11-06)
-------------------

23.2.0 (2023-10-09)
-------------------

23.1.0 (2023-10-04)
-------------------
* Add missing header required by the rclcpp::NodeOptions type (`#2324 <https://github.com/ros2/rclcpp/issues/2324>`_)
* Contributors: Ignacio Vizzo

23.0.0 (2023-09-08)
-------------------
* Update API docs links in package READMEs (`#2302 <https://github.com/ros2/rclcpp/issues/2302>`_)
* Contributors: Christophe Bedard

22.2.0 (2023-09-07)
-------------------

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
* Update all rclcpp packages to C++17. (`#2121 <https://github.com/ros2/rclcpp/issues/2121>`_)
* Contributors: Chris Lalancette

19.3.0 (2023-03-01)
-------------------

19.2.0 (2023-02-24)
-------------------

19.1.0 (2023-02-14)
-------------------

19.0.0 (2023-01-30)
-------------------
* Improve component_manager_isolated shutdown (`#2085 <https://github.com/ros2/rclcpp/issues/2085>`_)
* Contributors: Michael Carroll

18.0.0 (2022-12-29)
-------------------
* Update maintainers (`#2043 <https://github.com/ros2/rclcpp/issues/2043>`_)
* Contributors: Audrow Nash

17.1.0 (2022-11-02)
-------------------
* use unique ptr and remove unuseful container (`#2013 <https://github.com/ros2/rclcpp/issues/2013>`_)
* Contributors: Chen Lihui

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
