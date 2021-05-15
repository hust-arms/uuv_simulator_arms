/*                                                                           
 * Filename: uuv_gazebo_plugins/BodyUpdatePlugin.cc
 * Path: uuv_gazebo_plugins
 * Created Date: Friday, Janurary 29th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

#include <uuv_gazebo_plugins/BaseUpdatePlugin.hh>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(BaseUpdatePlugin)

/////////////////////////////////////////////////
BaseUpdatePlugin::BaseUpdatePlugin(){}

/////////////////////////////////////////////////
BaseUpdatePlugin::~BaseUpdatePlugin()
{
#if GAZEBO_MAJOR_VERSION >= 8
  this->updateConnection_.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection_);
#endif
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::Load(physics::ModelPtr _model, 
                            sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model != NULL, "Invalid model pointer");
    GZ_ASSERT(_sdf != NULL, "Invalid SDF element pointer");
    
    this->model_ = _model;
    this->world_ = _model->GetWorld();
    
    // stop physics dynamic of world
    if(this->world_->GetEnablePhysicsEngine()) 
    {
        this->world_->EnablePhysicsEngine(false);
    }

    // set node ptr 
    std::string worldName;
#if GAZEBO_MAJOR_VERSION >= 8
    worldName = this->world_->Name();
#else
    worldName = this->world_->GetName();
#endif
    this->node_->Init(worldName);

    // find base link
    if(_sdf->HasElement("link"))
    {
        for(sdf::ElementPtr linkElem = _sdf->GetElement("link"); linkElem; 
            linkElem = linkElem->GetNextElement("link"))
        {
            physics::LinkPtr link;
            std::string linkName = "";

            if(linkElem->HasAttribute("name"))
            {
                linkName = linkElem->Get<std::string>("name");
                std::size_t found = linkName.find("base_link");
                if(found != std::string::npos)
                {
                    this->baseLinkName_ = linkName;
                    gzmsg << "Name of the base link: " << this->baseLinkName_ << std::endl;
                }

                link = this->model_->GetLink(linkName);
                if(!Link)
                {
                    gzwarn << "Specified link [" << linkName << "] not found" << std::endl;
                    continue;
                }
            }
            else
            {
                gzwarn << "Attribute name missing from link [" << linkName << "]" << std::endl;
                continue;
            }
        }
    }

    // Connect the update event CB
    this->Connect()
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::Init()
{
    // Do nothing
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::Update(const common::UpdateInfo& _info)
{
    double time = _info.simTime.Double();

    link = this->model_->GetLink(this->baseLinkName_);

    link->SetLinkWorldPose(this->base_link_pose_);
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::Connect()
{
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&BaseUpdatePlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::UpdateBasePose(ConstPose3dPtr& _msg)
{
    this->base_link_pose_.Set(ignition::math::Vector3d(_msg.x(), _msg.y(), _msg.z()), 
                              ignition::math::Vector3d(_msg.roll(), _msg.pitch(), _msg.yaw()));

}

}; //ns
