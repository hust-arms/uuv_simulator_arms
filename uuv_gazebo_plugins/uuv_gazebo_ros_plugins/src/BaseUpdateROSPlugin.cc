/*                                                                           
 * Filename: uuv_gazebo_plugins/BodyUpdateROSPlugin.cc                       
 * Path: uuv_gazebo_plugins
 * Created Date: Friday, Janurary 29th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <uuv_gazebo_ros_plugins/BaseUpdateROSPlugin.hh>

#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>

namespace uuv_simulator_ros
{
/////////////////////////////////////////////////
BaseUpdateROSPlugin::BaseUpdateROSPlugin(){}

/////////////////////////////////////////////////
BaseUpdateROSPlugin::~BaseUpdateROSPlugin()
{
    this->rosNode_->shutdown();
}

/////////////////////////////////////////////////
void BaseUpdateROSPlugin::Load(gazebo::physics::NodePtr _parent, 
                               sdf::ElementPtr _sdf)
{
    if(!ros::isInitialized())
    {
        gzerr << "Not loading plugin since ROS has not been "
              << "properly initialized.  Try starting gazebo with ros plugin:\n"
              << "  gazebo -s libgazebo_ros_api_plugin.so\n";
        return;
    }

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
    this->posesub_ = this->rosNode_->subscribe<geometry_msgs::Pose>(
        _parent->GetName() + "/next_pose", 10, boost::bind(&BaseUpdateROSPlugin::UpdateBasePose, this, _1));

    this->nedTransform_.header.frame_id = this->model->GetName() + "/base_link";
    this->nedTransform_.child_frame_id = this->model->GetName() + "/base_linK_ned";
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
    this->tfBroadCaster_.sendTramsform(this->nedTransform_);
}

/////////////////////////////////////////////////A
void BaseUpdateROSPlugin::UpdateBasePose(const geometry_msgs::Pose::ConstPtr& _msg)
{
    tf::Quaternion q(_msg.quaternion.x, _msg.quaternion.y, _msg.quaternion.z, _msg.quaternion.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    this->baseLinkPose_.Set(_msg.position.x, _msg.position.y, _msg.position.z, roll, pitch, yaw);
}

}; // ns
