#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_INTERFACE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_INTERFACE_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <ignition/math.hh>

#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

// =================== ROS =====================//
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ros/subscribe_options.h"

//=============== ROS MSG TYPES ===============//


#include <nav_msgs/Odometry.h>

namespace gazebo {
// Default values

static const std::string kDefaultNamespace = "";

class GazeboOdometryPlugin : public ModelPlugin {
 public:
  GazeboOdometryPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
       
	ros_node_handle_(NULL) {}

  virtual ~GazeboOdometryPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr sdf);

  /// \brief Called when the model is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/, ros::Publisher& _pub);

 private:
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  std::string namespace_;
  physics::WorldPtr _world;
  physics::ModelPtr model;

  ros::NodeHandle* ros_node_handle_;
  ros::Publisher ros_pub_;
  nav_msgs::Odometry odom;

  double _pub_rate;

  common::Time _last_pub_time;
  common::Time _last_time;

  ignition::math::Pose3d _pose_model_start;

//   typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> GzWindSpeedMsgPtr;

//   void GzWindSpeedCallback(GzWindSpeedMsgPtr &_msg);

  

};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_INTERFACE_PLUGIN_H