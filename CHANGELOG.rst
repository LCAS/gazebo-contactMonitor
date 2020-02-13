^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package contact_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2020-02-13)
------------------
* Merge pull request `#3 <https://github.com/LCAS/gazebo-contactMonitor/issues/3>`_ from gpdas/melodic-devel
  gazebo8-* -> gazebo-*
* gazebo8-* -> gazebo-*
* Corrected wrong param name. Added params as args.
* Contributors: Gautham P Das, Manuel Fernandez-Carmona, gpdas

1.1.1 (2019-02-04)
------------------
* Added CMake rule to install launch scripts
* Contributors: Manuel Fernandez-Carmona

1.1.0 (2019-02-04)
------------------
* Topic namess follow the CCA rules. Added an error msg and 1s delay if no gz topic is found
* New README. Extra parameters added
* Update package.xml
  Added missing build dependence
* Contributors: Manuel Fernandez-Carmona

1.0.0 (2019-02-01)
------------------
* Merge pull request `#2 <https://github.com/LCAS/gazebo-contactMonitor/issues/2>`_ from pulver22/fix_gazebo_synch
  Fix gazebo synch
* Update contactMonitor.cpp
* Update contactMonitor.launch
* Update contactMonitor.cpp
  Do not create global vars unless necessary.
* Update contactMonitor.launch
  Don't remove functionallities
* Wait for gazebo generate the gz contacts topic before subscribe to it
* Wait for gazebo generate the gz contacts topic before subscribe to it
* Moved to spinOnce() to control the rate
* gazebo 8
* Merge pull request `#1 <https://github.com/LCAS/gazebo-contactMonitor/issues/1>`_ from pulver22/master
  Added full info to contact msgs + Clang refactor
* Added full contact information to ROS msg
* Added full contact information to ROS msg
* now publishing ALL collision names
* package renamed
* Update contactMonitor.cpp
* Update contactMonitor.launch
* Update README.md
* Initial commit
* Contributors: Manuel Fernandez-Carmona, Riccardo Polvara
