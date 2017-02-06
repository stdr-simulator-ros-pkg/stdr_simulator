^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stdr_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2017-02-06)
------------------
* Fix cmakes, no more cmake warnings

0.3.1 (2016-07-18)
------------------
* Add forgotten dependency on visualization_msgs

0.3.0 (2016-07-18)
------------------
* Add an interfaces test, as a functional test using rostest
* Fix synchronization bug on new robot registration
* Migrate to package format 2
* Visualization for custom sources using Marker messages (`#184 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/pull/184>`_)
* Remove unnecessary image_loader library in cmake
* Add forgotten run dependency to map_server (`#172 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/172>`_)
* Remove leading slash from frame_id

0.2.0 (2014-07-25)
------------------
* Feedback added in case of double frame ids
* Fixes `#148 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/148>`_
* Fix return values for service callbacks in stdr_server
* Server support for co2, sound, thermal sources
* Deletion of RFID tags supported
* Added rfid_tag insertion support for server

0.1.3 (2014-03-25)
------------------

0.1.2 (2014-03-09)
------------------

0.1.1 (2014-02-10)
------------------

0.1.0 (2014-02-06)
------------------
* first public release for Hydro
