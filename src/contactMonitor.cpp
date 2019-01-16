// based on a basic gazebo topic listener at:
// https://bitbucket.org/osrf/gazebo/src/256f98f2318bf2078e708f069367f1b71549ffb6/examples/stand_alone/listener/listener.cc
// mfc

#include <contactMonitor/contactMonitor.h>

ros::Publisher pub;
ros::Publisher fag_pub;
std::string robot_model_name ;
std::string actor_model_name ;
std::string collisions_topic_name ;
std::string collision_names_topic_name;


/////////////////////////////////////////////////
// Function is called everytime a message is received.
void contact_callback(ConstContactsPtr &_msg)
{
  // Dump the message contents to stdout.
  //std::cout << _msg->DebugString();
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
       fag_pub.publish(contact_data);

       if (( (collision1.find(robot_model_name) != std::string::npos) || (collision2.find(robot_model_name) != std::string::npos) ) &&
          ( (collision1.find(actor_model_name) != std::string::npos) || (collision2.find(actor_model_name) != std::string::npos) )) {
                gazebo::msgs::Time when = _msg->contact(i).time();
                tmstp.data.sec = when.sec();
                tmstp.data.nsec = when.nsec();
                pub.publish(tmstp);
                return;
           }
    }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{

  ros::init(_argc, _argv, "contactMonitor");
  ros::NodeHandle nh;

  // read configuration
  ros::param::param<std::string>("~collisions_topic_name", collisions_topic_name, "/collisions");
  ros::param::param<std::string>("~collision_names_topic_name", collision_names_topic_name, "/fag");

  ros::param::param<std::string>("~robot_model_name", robot_model_name, "robot1");
  ros::param::param<std::string>("~actor_model_name", actor_model_name, "actor1");

  pub = nh.advertise<std_msgs::Time>(collisions_topic_name, 5);
  fag_pub = nh.advertise<gazebo_msgs::ContactState>(collision_names_topic_name, 5);

  std_msgs::Time tmstp;

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", contact_callback);

  ros::spin();

  // Make sure to shut everything down.
  gazebo::transport::fini();
}
