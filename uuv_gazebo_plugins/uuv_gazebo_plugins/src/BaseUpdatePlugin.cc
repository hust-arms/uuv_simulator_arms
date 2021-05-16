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
BaseUpdatePlugin::BaseUpdatePlugin()
{
}

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
    gzmsg << "<BaseUpdatePlugin>: Load model" << std::endl; 

    GZ_ASSERT(_model != NULL, "Invalid model pointer");
    GZ_ASSERT(_sdf != NULL, "Invalid SDF element pointer");
    
    this->model_ = _model;
    this->world_ = _model->GetWorld();
    
    // stop physics dynamic of world
    this->world_->SetPhysicsEnabled(false);

    // set node ptr 
    this->node_ = transport::NodePtr(new transport::Node());
    
    std::string worldName;
#if GAZEBO_MAJOR_VERSION >= 8
    worldName = this->world_->Name();
#else
    worldName = this->world_->GetName();
#endif
    this->node_->Init(worldName);

    // set topics
    std::string poseTopic;
    if(_sdf->HasElement("base_link_state_topic"))
        poseTopic = _sdf->Get<std::string>("base_link_state_topic");
    else
        poseTopic = "/" + _model->GetName() + "/base_link_state";

    gzmsg << "<BaseUpdatePlugin>: base link pose topic: " << poseTopic << std::endl; 

    this->poseSub_ = this->node_->Subscribe(poseTopic, &BaseUpdatePlugin::UpdateBasePose, this);

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
                if(!link)
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
    this->Connect();
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::Init()
{
    // Do nothing
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::Update(const common::UpdateInfo& _info)
{
    physics::LinkPtr link = this->model_->GetLink(this->baseLinkName_);

    if(link)
        link->SetWorldPose(this->baseLinkPose_);
    // link->SetLinkWorldPose(this->baseLinkPose_);
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::Connect()
{
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&BaseUpdatePlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void BaseUpdatePlugin::UpdateBasePose(ConstPosePtr& _msg)
{
    printf("<BaseUpdatePlugin>: Update base pose\n");
    // this->baseLinkPose_.Set(_msg->position(), _msg->orientation());
    this->baseLinkPose_ = ignition::math::Pose3d(_msg->position().x(), _msg->position().y(), _msg->position().z(), 
                            _msg->orientation().w(), _msg->orientation().x(), _msg->orientation().y(), _msg->orientation().z());
}

}; //ns
