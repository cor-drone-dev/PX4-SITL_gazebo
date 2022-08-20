
#include "gazebo_ros_interface_plugin.h"
#include "common.h"
#include <math.h>

namespace gazebo {

GazeboRosInterfacePlugin::~GazeboRosInterfacePlugin() {
  update_connection_->~Connection();
  if (ros_node_handle_) {
	ros_node_handle_->shutdown();
	delete ros_node_handle_;
  };
}

void GazeboRosInterfacePlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
  world_ = world;

  std::cout << "[gazebo_ros_interface_plugin] Initialized gazebo ros interface plugin" << std::endl;

  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_ros_interface_plugin] Please specify a robotNamespace.\n";
  }

  // Gazebo node handle
  gz_node_handle_ = transport::NodePtr(new transport::Node());
  gz_node_handle_->Init(namespace_);

  // ROS node handle
  ros_node_handle_ = new ros::NodeHandle();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosInterfacePlugin::OnUpdate, this, _1));

  gz_wind_sub_ = gz_node_handle_->Subscribe<physics_msgs::msgs::Wind>("~/world_wind", &GazeboRosInterfacePlugin::GzWindSpeedCallback, this);
  ros_wind_pub_ = ros_node_handle_->advertise<geometry_msgs::Vector3Stamped>("/wind_speed", 1);
}

// This gets called by the world update start event.
void GazeboRosInterfacePlugin::OnUpdate(const common::UpdateInfo& _info) {

}

void GazeboRosInterfacePlugin::GzWindSpeedCallback(GzWindSpeedMsgPtr &_msg){
  geometry_msgs::Vector3Stamped wind_v;
  // Extract time
  int time_sec = round(_msg->time_usec()/1000000);
  int time_nsec = (_msg->time_usec() - time_sec*1000000)*1000;
  wind_v.header.stamp.sec = time_sec;
  wind_v.header.stamp.nsec = time_nsec;
  wind_v.vector.x = _msg->velocity().x();
  wind_v.vector.y = _msg->velocity().y();
  wind_v.vector.z = _msg->velocity().z();
  ros_wind_pub_.publish(wind_v);
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosInterfacePlugin);
}
