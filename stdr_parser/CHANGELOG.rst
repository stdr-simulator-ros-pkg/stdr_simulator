^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stdr_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2017-02-06)
------------------
* Fix cmakes, no more cmake warnings

0.3.1 (2016-07-18)
------------------

0.3.0 (2016-07-18)
------------------
* Migrate to package format 2
* Refactoring on parser node destruction (`#186 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/pull/186>`_)
* Allow loading local/custom resource files (`#178 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/178>`_)
* Fix bug on footprint loading (`#171 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/171>`_)
* Added kinematic model noise parameters support

0.2.0 (2014-07-25)
------------------
* Fixed `#148 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/148>`_
* tinyxml frees the memory itself (http://tinyxml.cvs.sourceforge.net/viewvc/tinyxml/tinyxml/tinyxml.cpp?revision=1.98&view=markup) so nothiing more to do. Closes `#111 <https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/111>`_
* Fixed partially the memory release of parser
* Parser support for new sensors. (thermal, sound, CO2, rfid)
* Fixed messageToFile, remains messageToXmlElement
* More support for YAML-CPP 0.5+

0.1.3 (2014-03-25)
------------------
* Load and save robot in robot creator works
* Add points parsing for footprints
  This adds support for parsing a list of points for footprints:
  * Allow multiple copies of the point tag ( only used in footprints )
  * Add a parser specialization for geometry_msgs::Point
  * Parse points if they appear in a footprint

0.1.2 (2014-03-09)
------------------

0.1.1 (2014-02-10)
------------------

0.1.0 (2014-02-06)
------------------
* first public release for Hydro
