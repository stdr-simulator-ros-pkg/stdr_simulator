^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stdr_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2017-02-06)
------------------
* Fix cmakes, no more cmake warnings
* Add forgotten install target for omni_motion_controller (`#195 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/195>`_)

0.3.1 (2016-07-18)
------------------

0.3.0 (2016-07-18)
------------------
* Migrate to package format 2
* Angles laser beams evenly (`#182 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/pull/182>`_)
* Fix laser scans off map bug (`#174 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/174>`_)
* Add support for kinematic model noise parameters
* Add support for omni directional motion model
* Fix segfault on robot_manager due to uninitialized pointer (`#159 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/159>`_)

0.2.0 (2014-07-25)
------------------
* Added CO2 sensors
* Added sound sensors
* Added thermal sensors, they return the larger temperature in their span
* Refactoring on stdr_robot. Removed duplicate code from derived classes.
* Rfid Reader robot sensor added.

0.1.3 (2014-03-25)
------------------
* Publish odometry
* Collision detection works decently with random shaped robots
* Fixed bug preventing robot from turning after collision with obstacle
* Removed robot's erroneous random collisions in obstacle-free space. `#133 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/133>`_
* Use footprint points if available.

0.1.2 (2014-03-09)
------------------
* add collision checking
* fix minor issues with robot reposition

0.1.1 (2014-02-10)
------------------
* fix segfault when robot goes out of map bounds

0.1.0 (2014-02-06)
------------------
* first public release for Hydro
