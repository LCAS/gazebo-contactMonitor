// based on a basic gazebo topic listener at:
// https://bitbucket.org/osrf/gazebo/src/256f98f2318bf2078e708f069367f1b71549ffb6/examples/stand_alone/listener/listener.cc
// mfc

#include <contactMonitor/contactMonitor.h>

ros::Publisher pub;
ros::Publisher fag_pub;
std::string robot_model_name;
std::string actor_model_name;
std::string collisions_topic_name;
std::string collision_names_topic_name;

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void contact_callback(ConstContactsPtr& _msg)
{
  // Dump the message contents to stdout.

  std::string collision1;
  std::string collision2;
  std_msgs::Time tmstp;

  gazebo_msgs::ContactState contact_data;

  // Iterate over all the contacts in the message
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    collision1 = _msg->contact(i).collision1();
    collision2 = _msg->contact(i).collision2();

    contact_data.collision1_name = collision1;
    contact_data.collision2_name = collision2;
    contact_data.wrenches.clear();
    contact_data.contact_positions.clear();
    contact_data.contact_normals.clear();
    contact_data.depths.clear();

    // sum up all wrenches for each DOF
    geometry_msgs::Wrench total_wrench;
    total_wrench.force.x = 0;
    total_wrench.force.y = 0;
    total_wrench.force.z = 0;
    total_wrench.torque.x = 0;
    total_wrench.torque.y = 0;
    total_wrench.torque.z = 0;

    unsigned int contactGroupSize = _msg->contact(i).position_size();
    for (unsigned int j = 0; j < contactGroupSize; ++j)
    {
      // Get force, torque and rotate into user specified frame.
      // frame_rot is identity if world is used (default for now)
      ignition::math::Quaterniond frame_rot;
      ignition::math::Vector3d frame_pos;
      ignition::math::Vector3d force =
          frame_rot.RotateVectorReverse(ignition::math::Vector3d(
              _msg->contact(i).wrench(j).body_1_wrench().force().x(),
              _msg->contact(i).wrench(j).body_1_wrench().force().y(),
              _msg->contact(i).wrench(j).body_1_wrench().force().z()));
      ignition::math::Vector3d torque =
          frame_rot.RotateVectorReverse(ignition::math::Vector3d(
              _msg->contact(i).wrench(j).body_1_wrench().torque().x(),
              _msg->contact(i).wrench(j).body_1_wrench().torque().y(),
              _msg->contact(i).wrench(j).body_1_wrench().torque().z()));

      // set wrenches
      geometry_msgs::Wrench wrench;
      wrench.force.x = force.X();
      wrench.force.y = force.Y();
      wrench.force.z = force.Z();
      wrench.torque.x = torque.X();
      wrench.torque.y = torque.Y();
      wrench.torque.z = torque.Z();
      contact_data.wrenches.push_back(wrench);

      total_wrench.force.x += wrench.force.x;
      total_wrench.force.y += wrench.force.y;
      total_wrench.force.z += wrench.force.z;
      total_wrench.torque.x += wrench.torque.x;
      total_wrench.torque.y += wrench.torque.y;
      total_wrench.torque.z += wrench.torque.z;

      // transform contact positions into relative frame
      // set contact positions
      ignition::math::Vector3d position = frame_rot.RotateVectorReverse(
          ignition::math::Vector3d(_msg->contact(i).position(j).x(),
                                   _msg->contact(i).position(j).y(),
                                   _msg->contact(i).position(j).z()) -
          frame_pos);
      geometry_msgs::Vector3 contact_position;
      contact_position.x = position.X();
      contact_position.y = position.Y();
      contact_position.z = position.Z();
      contact_data.contact_positions.push_back(contact_position);

      // rotate normal into user specified frame.
      // frame_rot is identity if world is used.
      ignition::math::Vector3d normal =
          frame_rot.RotateVectorReverse(ignition::math::Vector3d(
              _msg->contact(i).normal(j).x(), _msg->contact(i).normal(j).y(),
              _msg->contact(i).normal(j).z()));
      // set contact normals
      geometry_msgs::Vector3 contact_normal;
      contact_normal.x = normal.X();
      contact_normal.y = normal.Y();
      contact_normal.z = normal.Z();
      contact_data.contact_normals.push_back(contact_normal);

      // set contact depth, interpenetration
      contact_data.depths.push_back(_msg->contact(i).depth(j));
    }

    fag_pub.publish(contact_data);

    if (((collision1.find(robot_model_name) != std::string::npos) ||
         (collision2.find(robot_model_name) != std::string::npos)) &&
        ((collision1.find(actor_model_name) != std::string::npos) ||
         (collision2.find(actor_model_name) != std::string::npos)))
    {
      gazebo::msgs::Time when = _msg->contact(i).time();
      tmstp.data.sec = when.sec();
      tmstp.data.nsec = when.nsec();
      pub.publish(tmstp);
      return;
    }
  }
}

/////////////////////////////////////////////////
int main(int _argc, char** _argv)
{

  ros::init(_argc, _argv, "contactMonitor");
  ros::NodeHandle nh;

  // read configuration
  ros::param::param<std::string>("~collisions_topic_name",
                                 collisions_topic_name, "/collisions");
  ros::param::param<std::string>("~collision_names_topic_name",
                                 collision_names_topic_name, "/fag");

  ros::param::param<std::string>("~robot_model_name", robot_model_name,
                                 "robot1");
  ros::param::param<std::string>("~actor_model_name", actor_model_name,
                                 "actor1");

  pub = nh.advertise<std_msgs::Time>(collisions_topic_name, 5);
  fag_pub =
      nh.advertise<gazebo_msgs::ContactState>(collision_names_topic_name, 5);


  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Check if the topic to subscribe is present within the list, and
  // create the subscriber only later
  bool topic_found = false;
  while (topic_found == false)
  {
    std::list<std::string> topics_list =
        gazebo::transport::getAdvertisedTopics("");
    for (std::list<std::string>::iterator topic = topics_list.begin();
         topic != topics_list.end(); topic++)
    {
      std::cout << (*topic) << "\n";
      if ((*topic).compare("/gazebo/default/physics/contacts") == 0)
      {
        std::cout<<"Topic found!"<<std::endl;
        topic_found = true;
      }
      else
      {
        std::cout << "Check next topic" << std::endl;
      }
    }
  }

  gazebo::transport::SubscriberPtr sub =
      node->Subscribe("/gazebo/default/physics/contacts", contact_callback);

  ros::spin();

  // Make sure to shut everything down.
  gazebo::transport::fini();
}
