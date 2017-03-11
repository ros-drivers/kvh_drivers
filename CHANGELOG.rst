^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kvh
^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2017-03-11)
------------------
* fixed pkg name for launch file
* added check for testing to remove builder warning
* added auto generated wiki page for documentation
* Contributors: Geoff Viola, geoffviola

1.0.2 (2016-09-10)
------------------
* relaxed errors as warnings
* Contributors: Geoff Viola

1.0.1 (2016-08-13)
------------------
* added installation path for library
* quiting starting warning
* unit tested driver
* roslinted all files
* clang format
* Merge pull request `#2 <https://github.com/ros-drivers/kvh_drivers/issues/2>`_ from geoffviola/master
  Removed bad cereal API call bug
* fixed author tag
* change the maintainer and author
* CMake cleanup and reverted back to kvh package
* Update README.md
* fixes for kinetic
* added invert option and changed topic name
* tested on real dsp3000
* added configuration modes via parameters
* formatted file
* added a larger buffer for a slower processor
* consistently reading
* debugging some changes
* Initial untested Indigo release
* the port defined in the launch file now passes properly.
* Expanded cereal_port README file
* Removed a bunch of old .svn directories
* Added header info to dsp3000.cpp
* Output changed from degrees to radians.
* Add the needed add_boost_directories macro.
* Fixed _pub name and TIMEOUT definition
* Merge branch 'master' of https://bitbucket.org/clearpathrobotics/kvh
* Make package build against local copy of cereal_port.
* Remove unneeded stuff from cereal_port.
* removed the scripts directory
* Initial "write" to device made more efficient, exception has a better description.
* Added ROS_DEBUG output
* Changed TIMEOUT from define to const
* Deleted some temp files
* Added .gitignore
* Renamed "flavour" directory to "scripts"
* deleted build files
* Initial push containing entire DSP-3000 ROS node.
* Initial commit
* Contributors: Geoff Viola, Jeff Schmidt, Mike Purvis, geoffviola, jeff-o, sbir
