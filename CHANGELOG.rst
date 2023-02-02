^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2023-02-02)
------------------
* Added ExecuteProcess action to set the domain ID at launch
* Added robot_upstart dependency
* Added dependencies.repos file
* Contributors: Roni Kreinin

1.0.0 (2022-10-16)
------------------
* Fixed copyright
  Added diagnostic dependencies
* Ignore __pycache\_\_
* Added jackal_diagnostics
* Added install and uninstall scripts for robot_upstart job
  Launch file cleanup
* Added exec dependencies
* Added accessories launch
* Minor hardware fix
* Consolidated jackal_base and jackal_bringup into one package 'jackal_robot'
* Velocity mode when 0 velocity is commanded
* Renamed JackalBase to JackalHardwareInterface
  Reverted package version and description to current values
  Use /dev/jackal as the micro ros serial device
* Removed unused variables.
  Updated copyright.
* rclcpp::spin_some before checking for feedback
* Mutex on feedback message
* Working ros2_control
* Add dependency for jackal_tests (`#54 <https://github.com/jackal/jackal_robot/issues/54>`_)
  This PR should only be merged once the test package is released.
* Add Blackfly to accessories launch
* Added spinnaker_camera_driver to package.xml
* Contributors: Joey Yang, Luis Camero, Roni Kreinin

0.7.4 (2022-03-11)
------------------

0.7.3 (2022-03-08)
------------------

0.7.2 (2022-02-15)
------------------

0.7.1 (2022-01-18)
------------------

0.7.0 (2021-04-23)
------------------
* Merge development changes from live hardware test into noetic-devel
* Contributors: Chris I-B, Chris Iverach-Brereton

0.6.1 (2021-03-08)
------------------

0.6.0 (2020-04-20)
------------------

0.5.1 (2020-04-14)
------------------

0.3.9 (2019-06-19)
------------------

0.3.8 (2018-11-07)
------------------

0.3.7 (2018-08-02)
------------------

0.3.6 (2016-09-30)
------------------

0.3.5 (2016-02-22)
------------------

0.3.4 (2016-02-10)
------------------

0.3.3 (2015-02-20)
------------------

0.3.2 (2015-02-19)
------------------

0.3.1 (2015-02-03)
------------------

0.3.0 (2015-01-20)
------------------

0.2.2 (2015-01-14)
------------------

0.2.1 (2015-01-12)
------------------

0.2.0 (2015-01-12)
------------------
* Add jackal_bringup to robot metapackage.
* Contributors: Mike Purvis

0.1.0 (2014-11-11)
------------------
* Add jackal_robot metapackage.
* Contributors: Mike Purvis
