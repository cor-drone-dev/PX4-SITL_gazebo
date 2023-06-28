#include "gazebo_odometry_plugin.h"
#include "common.h"
#include <math.h>

namespace gazebo {

GazeboOdometryPlugin::~GazeboOdometryPlugin() {
  if (ros_node_handle_) {
	ros_node_handle_->shutdown();
	delete ros_node_handle_;
  };
}

void GazeboOdometryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr sdf) {
  model = _model;

  _world = model->GetWorld();
  _last_time = _world->SimTime();
  _last_pub_time = _world->SimTime();
  // remember start pose -> VIO should always start with zero
  _pose_model_start = model->WorldPose();

  // ROS node handle
  ros_node_handle_ = new ros::NodeHandle();
  ros_pub_ = ros_node_handle_->advertise<nav_msgs::Odometry>("/px4_sitl_odom", 1);
  _pub_rate = 20;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo& _info) {
        this->OnUpdate(_info, ros_pub_);
    });

  
}

// This gets called by the world update start event.
void GazeboOdometryPlugin::OnUpdate(const common::UpdateInfo& _info, ros::Publisher& _pub) {
  
  common::Time current_time = _world->SimTime();

  double dt = (current_time - _last_pub_time).Double();

  if (dt > 1.0 / _pub_rate) {

    // get pose of the model that the plugin is attached to
    ignition::math::Pose3d pose_model_world = model->WorldPose();
    ignition::math::Vector3d velocity_model_world = model->WorldLinearVel();
    ignition::math::Vector3d angular_velocity_model = model->RelativeAngularVel();

    ignition::math::Pose3d pose_model; // pose in local frame (relative to where it started)
    pose_model.Pos().X() = pose_model_world.Pos().X() - _pose_model_start.Pos().X();
    pose_model.Pos().Y() = pose_model_world.Pos().Y() - _pose_model_start.Pos().Y();
    pose_model.Pos().Z() = pose_model_world.Pos().Z() - _pose_model_start.Pos().Z();
    pose_model.Rot().Euler(pose_model_world.Rot().Roll(),
                           pose_model_world.Rot().Pitch(),
                           pose_model_world.Rot().Yaw());
    

    // Fill in the header
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom_frame";

    // Fill in the child frame ID
    odom.child_frame_id = "base_link";

    // Fill in the pose information
    odom.pose.pose.position.x = pose_model.Pos().X();
    odom.pose.pose.position.y = pose_model.Pos().Y();
    odom.pose.pose.position.z = pose_model.Pos().Z();

    ignition::math::Quaterniond pose_model_quaternion = pose_model.Rot();
    odom.pose.pose.orientation.x = pose_model_quaternion.X();
    odom.pose.pose.orientation.y = pose_model_quaternion.Y();
    odom.pose.pose.orientation.z = pose_model_quaternion.Z();
    odom.pose.pose.orientation.w = pose_model_quaternion.W();

    // Fill in the twist information
    odom.twist.twist.linear.x = velocity_model_world.X();
    odom.twist.twist.linear.y = velocity_model_world.Y();
    odom.twist.twist.linear.z = velocity_model_world.Z();

    odom.twist.twist.angular.x = angular_velocity_model.X();
    odom.twist.twist.angular.y = angular_velocity_model.Y();
    odom.twist.twist.angular.z = angular_velocity_model.Z();
    // msg.data = true;


    _pub.publish(odom);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboOdometryPlugin);
}