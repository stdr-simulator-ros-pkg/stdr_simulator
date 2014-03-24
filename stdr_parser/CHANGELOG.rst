^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stdr_parser
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
