# gazebo-contactMonitor

Monitors collisions in gazebo by checking (gazebo) topic /gazebo/default/physics/contacts" and then republishes into a ROS topic the collision timestamp.
To check that two models are colliding we provide a keyword for each models name.
See launchfile for params.
