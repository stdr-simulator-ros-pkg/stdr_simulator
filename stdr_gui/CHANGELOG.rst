^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stdr_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2017-02-06)
------------------
* Fix cmakes, no more cmake warnings

0.3.1 (2016-07-18)
------------------

0.3.0 (2016-07-18)
------------------
* Migrate to package format 2, fix dependency issues
* Refactoring in sources classes, using better astraction
* Support for omni motion controller in robot creator
* Kinematic model appears in info panel

0.2.0 (2014-07-25)
------------------
* Feedback added in case of double frame ids
* Fixes `#140 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/140>`_
* Added zoom movement by keyboard arrows.
* Fixes `#146 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/146>`_
* GUI application now closes with SINGINT
* CO2 sensors work
* Sound sensors working
* Thermal sensors are working. They return the larger temperature in their span
* Added dummy gui functions for sound sources
* Full RFID tag and readers support
* RFID reader visibility status works
* Rfid antennas visible in information tree
* RFID reader sensor works.
* Deletion of RFID tags supported
* Added click event in GUI for rfid tags
* Changed the robot label size to be equal to the frame_id size
* Rfid tags inserted from GUI and shown with their ids

0.1.3 (2014-03-25)
------------------
* fixed `#143 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/143>`_
* fixed `#142 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/142>`_
* fixed `#125 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/125>`_
* fixed `#141 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/141>`_
* Fixed glob in CMakeLists.txt
* Load and save robot in robot creator works
* Robot creator works with footprint points
* Zoom in GUI is now performed with mouse wheel
* Added antialiasing to robot draw in gui
* Polygon robots appear in gui as they should.

0.1.2 (2014-03-09)
------------------
* Minor changes. Added message when relocation is not possible.
* Different labels for add and create robot

0.1.1 (2014-02-10)
------------------

0.1.0 (2014-02-06)
------------------
* first public release for Hydro
