
# Changelog
All notable changes to this project will be documented in this file.
Add your changes to the 'master' section. Try to keep it short and precise.
Add ticket numbers to all your change notes.

## [master]

### Added

### Changed

### Removed

## [1.0.0] - 2023.08.09

### Added
* Release v1.0.0

## [0.7.2] - 2023.08.08

### Changed
* Fixed the version request from git

## [0.7.1] - 2023.08.08

### Added
* Added support for publishing to github

### Changed
* Updated the documentation

## [0.7.0] - 2023.08.04

### Added
* [#48319] Add flags for publishing intensity and point_id
* [#48319] Added frame reader and publisher for Qb2 in Qb2LidarRos class
* [#48319] Added frame to point cloud converter in utils
* [#48319] Added an empty protocol directory to follow module directory structure
* [#48319] Added clang format file
* [#50606] Add support for GET API to get a snapshot from core-processing
* [#49683] Added documentation for live and snapshot driver
* [#49683] Added logging for parameters set by client
* [#48319] Added support for build against blickfeld-qb2 protocol from github repo
* [#49683] Add license
* Added support for humble

### Changed
* [#48319] Updated the driver and snapshot driver to use the unified Qb2LidarRos class for reading stream or single point cloud and publishing
* [#48319] Updated and cleaned up launch and scripts
* [#48319] Adjust the Cmake based on modification to driver and snapshot driver
* [#49683] General cleanup and improvements

### Removed
* [#48319] Removed the visibility control

## [0.6.0] - 2023.07.28

### Added
* [#49879] Allow for secure connection in snapshot driver
* [#50232] Add service to trigger service to Snapshot Driver
* [#49879] Allow for secure connection in driver

### Changed
* Update internal dependency versions

### Removed
* [#49879] Remove the port configuration

## [0.5.2] - 2023.06.30

### Changed
* Update core processing due to failing PCIe Debian package

## [0.5.1] - 2023.06.30

### Changed
* [#49797] Adapt to breaking API changes of core processing

## [0.5.0] - 2023.06.27

### Added
* [#49667] Added snapshot driver

## [0.4.0] - 2023.05.24

### Changed
* [#48319] Move the driver to blickfeld-software group and update it accordingly

## [0.3.0] - 2022.11.30

### Added
* Infrastructure for tests and a dummy test

### Changed
* [refs #44253] Adapt to API changes in core_processing

## [0.2.4] - 2022.11.15

### Added
* Add retry logic to socket connection

### Changed
* Unified coding style

## [0.2.3] - 2022.09.06

### Added
* Add Intensity channel to output pointcloud

### Removed
* Remove range output

## [0.2.2] - 2022.06.14

### Changed
* Update daedalus-base to blickfeld-base

## [0.2.1] - 2022.02.24

### Changed
* [refs #36242] refactor project, rename ROS package to daedalus_ros2_driver

## [0.2.0] - 2022.02.01

### Added
* [refs #35556] separate connection method with addition parameters

## [0.1.2] - 2022.01.10

### Changed
* Fix: make driver able to stop

## [0.1.1] - 2022.01.07

### Changed
* Fix: changed plugin name to blickfeld::ros_interop::DaedalusDriver in cmake
* Fix: move spin method to Thread to allow proper loading of component

## [0.1.0] - 2021.12.15

### Added
* [refs #34644] initial implementation

## [0.0.1] - xxxx.xx.xx

### Added
* Initial changelog
