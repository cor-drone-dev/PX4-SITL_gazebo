#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_INTERFACE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_INTERFACE_PLUGIN_H

#include <string>
#include <random>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Wind.pb.h"

// =================== ROS =====================//
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "ros/subscribe_options.h"

//=============== ROS MSG TYPES ===============//
#include <geometry_msgs/Vector3.h>

namespace gazebo {
// Default values
static const std::string kDefaultGzWindSubTopic = "~/world_wind";
static const std::string kDefaultNamespace = "";

class GazeboRosInterfacePlugin : public WorldPlugin {
 public:
  GazeboRosInterfacePlugin()
      : WorldPlugin(),
        namespace_(kDefaultNamespace),
        gz_node_handle_(NULL),
	ros_node_handle_(NULL) {}

  virtual ~GazeboRosInterfacePlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;

  std::string namespace_;

  transport::NodePtr gz_node_handle_;
  transport::SubscriberPtr gz_wind_sub_;

  ros::NodeHandle* ros_node_handle_;
  ros::Publisher ros_wind_pub_;

  typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> GzWindSpeedMsgPtr;

  void GzWindSpeedCallback(GzWindSpeedMsgPtr &_msg);

  physics_msgs::msgs::Wind wind_msg;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_INTERFACE_PLUGIN_H
