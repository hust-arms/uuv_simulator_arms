/*                                                                           
 * Filename: uuv_gazebo_plugins/BodyUpdateROSPlugin.cc                       
 * Path: uuv_gazebo_plugins
 * Created Date: Friday, Janurary 29th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <uuv_gazebo_ros_plugins/BaseUpdateROSPlugin.hh>

#include <tf/tf.h>
#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>

namespace uuv_simulator_ros
{
/////////////////////////////////////////////////
BaseUpdateROSPlugin::BaseUpdateROSPlugin()
{
}

/////////////////////////////////////////////////
BaseUpdateROSPlugin::~BaseUpdateROSPlugin()
{
    this->rosNode_->shutdown();
}

/////////////////////////////////////////////////
void BaseUpdateROSPlugin::Load(gazebo::physics::ModelPtr _parent, 
                               sdf::ElementPtr _sdf)
{
    if(!ros::isInitialized())
    {
        gzerr << "Not loading plugin since ROS has not been "
              << "properly initialized.  Try starting gazebo with ros plugin:\n"
              << "  gazebo -s libgazebo_ros_api_plugin.so\n";
        return;
    }

    gzmsg << "<BaseUpdateROSPlugin>: Load plugin\n";

    this->rosNode_.reset(new ros::NodeHandle(""));

    try
    {
        BaseUpdatePlugin::Load(_parent, _sdf);
    }
    catch(gazebo::common::Exception& _e)
    {
        gzerr << "Error loading plugin."
              << "Please ensure that your model is correct."
              << '\n';
        return;
    }

    // subscribe pose
    this->baseLinkStatesSub_ = this->rosNode_->subscribe<geometry_msgs::Pose>(
        _parent->GetName() + "/base_link_states", 10, boost::bind(&BaseUpdateROSPlugin::UpdateBasePose, this, _1));

    this->nedTransform_.header.frame_id = this->model_->GetName() + "/base_link";
    this->nedTransform_.child_frame_id = this->model_->GetName() + "/base_linK_ned";
    this->nedTransform_.transform.translation.x = 0;
    this->nedTransform_.transform.translation.y = 0;
    this->nedTransform_.transform.translation.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, 0);
    this->nedTransform_.transform.rotation.x = quat.x();
    this->nedTransform_.transform.rotation.y = quat.y();
    this->nedTransform_.transform.rotation.z = quat.z();
    this->nedTransform_.transform.rotation.w = quat.w();
}

/////////////////////////////////////////////////A
void BaseUpdateROSPlugin::Init()
{
    BaseUpdatePlugin::Init();
}

/////////////////////////////////////////////////A
void BaseUpdateROSPlugin::Reset()
{
}

/////////////////////////////////////////////////A
void BaseUpdateROSPlugin::Update(const gazebo::common::UpdateInfo& _info)
{
    BaseUpdatePlugin::Update(_info);
    this->nedTransform_.header.stamp = ros::Time::now();
    this->tfBroadcaster_.sendTransform(this->nedTransform_);
}

/////////////////////////////////////////////////A
void BaseUpdateROSPlugin::UpdateBasePose(const geometry_msgs::Pose::ConstPtr& _msg)
{
    double x, y, z;
    x = _msg->position.x; y = _msg->position.y; z = _msg->position.z;
    tf::Quaternion q(_msg->orientation.x, _msg->orientation.y, _msg->orientation.z, _msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    printf("<BaseUpdateROSPlugin>: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f\n", x, y, z, roll, pitch, yaw);
    this->baseLinkPose_.Set(x, y, z, roll, pitch, yaw);
}

}; // ns
