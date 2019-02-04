# gazebo-contactMonitor

This node has two functionalities:
- Forwards collisions between any two models in gazebo (at least one model must be dynamic).
- Monitors collisions between two given models then republishes its timestamp in ROS.

Parameters (see launch file for an example):
- `gazebo_physics_contact_topic_name`: gazebo topic where contacts are published. 
Usually follows the pattern `/gazebo/` \[world name\] `/physics/contacts`. 
Check your world file to find out the world name. Or check published gazebo topics with `gz topic -l`

- `collisions_timestamp_topic_name`: ROS topic where collision timestamps will be published. 
- `robot_model_name`: first model name to consider in monitored collisions published in above topic. It is used as a wildcard, partial names can be used. 
- `actor_model_name`: second model name. 
- `collision_names_topic_name`: All collisions are published in this topic name. 

