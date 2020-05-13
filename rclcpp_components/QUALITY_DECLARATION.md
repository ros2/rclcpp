This document is a declaration of software quality for the `rclcpp_components` package, based on the guidelines in [REP-2004](https://github.com/ros-infrastructure/rep/blob/master/rep-2004.rst).

# `rclcpp_components` Quality Declaration

The package `rclcpp_components` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Quality Categories in REP-2004](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-quality-categories) of the ROS2 developer guide.

## Version Policy [1]

### Version Scheme [1.i]

`rclcpp_components` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning)

### Version Stability [1.ii]

`rclcpp_components` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

All installed headers are in the `include` directory of the package, headers in any other folders are not installed and considered private.

### API Stability Policy [1.iv]

`rclcpp_components` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Policy [1.v]

`rclcpp_components` contains C++ code and therefore must be concerned with ABI stability, and will maintain ABI stability within a ROS distribution.

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

`rclcpp_components` will not break API nor ABI within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

## Change Control Process [2]

`rclcpp_components` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process).

### Change Requests [2.i]

All changes will occur through a pull request, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy. More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed, check [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#change-control-process) for additional information.

### Continuous Integration [2.iv]

All pull requests must pass CI on all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers)

Currently nightly results can be seen here:

* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

###  Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rclcpp_components` does not have a documented feature list.

### Public API Documentation [3.ii]

`rclcpp_components` does not cover a public API documentation.

The API is publicly available in the [ROS 2 API documentation](http://docs.ros2.org/latest/api/rclcpp_components/).

### License [3.iii]

The license for `rclcpp_components` is Apache 2.0, and a summary is in each source file, the type is declared in the [`package.xml`](./package.xml) manifest file, and a full copy of the license is in the [`LICENSE`](../LICENSE) file.

There is an automated test which runs a linter that ensures each file has a license statement. [Here](http://build.ros2.org/view/Epr/job/Epr__rclcpp_components__ubuntu_bionic_amd64/lastBuild/testReport/rclcpp_components/) can be found a list with the latest results of the various linters being run on the package.

### Copyright Statements [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rclcpp_components`.

There is an automated test which runs a linter that ensures each file has at least one copyright statement. Latest linter result report can be seen [here](http://build.ros2.org/view/Epr/job/Epr__rclcpp_components__ubuntu_bionic_amd64/lastBuild/testReport/rclcpp_components/copyright/).

## Testing [4]

### Feature Testing [4.i]

Each feature in `rclcpp_components` has corresponding tests which simulate typical usage, and they are located in the [`test`](https://github.com/ros2/rclcpp_components/tree/master/test) directory.
New features are required to have tests before being added.

Currently nightly test results can be seen here:

* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.
The tests aim to cover both typical usage and corner cases, but are quantified by contributing to code coverage.

### Coverage [4.iii]

`rclcpp_components` follows the recommendations for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#code-coverage), and opts to use line coverage instead of branch coverage.

This includes:

- tracking and reporting line coverage statistics
- achieving and maintaining a reasonable branch line coverage (90-100%)
- no lines are manually skipped in coverage calculations

Changes are required to make a best effort to keep or increase coverage before being accepted, but decreases are allowed if properly justified and accepted by reviewers.

### Performance [4.iv]

It is not yet defined if this package requires performance testing and how addresses this topic.

### Linters and Static Analysis [4.v]

`rclcpp_components` uses and passes all the ROS2 standard linters and static analysis tools for a C++ package as described in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters-and-static-analysis). Passing implies there are no linter/static errors when testing against CI of supported platforms.

Currently nightly test results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/rclcpp_components/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/rclcpp_components/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/rclcpp_components/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/rclcpp_components/)

## Dependencies [5]

Below are evaluations of each of `rclcpp_components`'s run-time and build-time dependencies that have been determined to influence the quality.

It has several "buildtool" dependencies, which do not affect the resulting quality of the package, because they do not contribute to the public library API.
It also has several test dependencies, which do not affect the resulting quality of the package, because they are only used to build and run the test code.

### Direct and Optional Runtime ROS Dependencies [5.i]/[5.ii]

`rclcpp_components` has the following runtime ROS dependencies:

#### `ament_index_cpp`

The `ament_index_cpp` package provides a C++ API to access the ament resource index.

It is **Quality Level 4**, see its [Quality Declaration document](https://github.com/ament/ament_index/blob/master/ament_index_cpp/QUALITY_DECLARATION.md).

#### `class_loader`

The `class_loader` package provides a ROS-independent package for loading plugins during runtime

It is **Quality Level 4**, see its [Quality Declaration document](https://github.com/ros/class_loader/blob/ros2/QUALITY_DECLARATION.md).

#### `composition_interfaces`

The `composition_interfaces` package contains message and service definitions for managing composable nodes in a container process.

It is **Quality Level 4**, see its [Quality Declaration document](https://github.com/ros2/rcl_interfaces/blob/master/composition_interfaces/QUALITY_DECLARATION.md).

#### `rclcpp`

The `rclcpp` package provides the ROS client library in C++.

It is **Quality Level 4**, see its [Quality Declaration document](https://github.com/ros2/rclcpp/blob/master/rclcpp/QUALITY_DECLARATION.md).

### Direct Runtime non-ROS Dependency [5.iii]

`rclcpp_components` has no run-time or build-time dependencies that need to be considered for this declaration.

## Platform Support [6]

`rclcpp_components` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), and tests each change against all of them.

Currently nightly build status can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/rclcpp_components/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/rclcpp_components/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/rclcpp_components/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/rclcpp_components/)

## Security

### Vulnerability Disclosure Policy [7.i]

This package does not yet have a Vulnerability Disclosure Policy
