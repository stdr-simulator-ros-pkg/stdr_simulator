^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stdr_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
