^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tracking_pid
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

------------------
* Add Apache 2.0 as license as per ROSIN
* Add backwards compatibility to ROS-Kinetic by overloading planner initialize function.
* Add ~loop param to make interpolator loop
* Deal with 0.0 velocity in Python path_interpolator
* Ported path_interpolator.py/PathInterpolator itself to path_interpolator.cpp and integrating it
* Re-format code according to ROS standard (https://github.com/davetcoleman/roscpp_code_format)

------------------
* Drive backwards when control point is behind robot (negative)
* Deal with paths that have duplicate poses

------------------
* Added dynamic reconfigure parameter for velocity
* Add options for different start-time than current time to allow for faster resuming after paused state
* Do not re-initialise interpolator on every pause callback but only when paused

------------------
* Publish when the path is done tracking
* The functionality of the Interpolator would also be suitable for an ActionServer, called with a path
* Add node to interpolate nav_msgs/Path and send that to the tracking_pid node
* Added interpolation between data points to allow for paths with data points further apart in time and space
* Add parameter to allow chosing controller frames

------------------
* Added backwards driving compatibility to the controller
* Fixed feedforward issue and some improvements in the perf_logger
* Added performance logger wrt to future lifetime tests
* Added general launch file in which desired trajectory can be set. Added dynamic reconfigure for the global_planner to change trajectories online easily
* Bugfixes and dynamic reconfigure of local planner
* Added feedforward actions, improved overall performance, bug fixes
