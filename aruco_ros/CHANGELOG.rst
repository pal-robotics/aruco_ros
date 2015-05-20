^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2015-05-20)
------------------
* More accurate ROS timestamps (callback triggering time)
  This commit ensures that:
  - all published msgs in a callback have the same timestamp
  - the time is as close as possible to the frame grabbing time (as fast as the marker detection may be, the delay might affect TF interpolation in an unacceptable way for applications like visual servoing)
* Install marker_publisher executable
  This target was missing in the installation rule
* Finished some renaming
* changes to finish branch merge
* aruco_ros: Fixing superfluous (and broken) linker arg to -laruco
* Reorganize aruco_ros into 3 packages
* Contributors: Bence Magyar, Jordi Pages, Josh Langsfeld, ObiWan, Steve Vozar
