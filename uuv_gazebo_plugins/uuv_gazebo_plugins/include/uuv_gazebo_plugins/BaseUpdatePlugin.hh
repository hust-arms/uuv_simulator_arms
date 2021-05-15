/*                                                                                             
 * Filename: uuv_gazebo_plugins/BodyUpdatePlugin.hh
 * Path: uuv_gazebo_plugins
 * Created Date: Friday, Janurary 29th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef __UUV_GAZEBO_PLUGINS_UNDERWATER_OBJECT_HH__
#define __UUV_GAZEBO_PLUGINS_UNDERWATER_OBJECT_HH__

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
/**
 * @brief Gazebo plugin class for UUV base status update
 */
class BaseUpdatePlugin : public gazebo::ModelPlugin
{
public:
    /**
     * @brief Constructor
     */
    BaseUpdatePlugin();

    /**
     * @brief Destructor
     */
    virtual ~BaseUpdatePlugin();

    /**
     * @brief Model load
     */
    virtual void Load(gazebo::physics::ModelPtr _model, 
                      sdf::ElementPtr _sdf);

    /**
     * @brief Initialization
     */
    virtual void Init();

    /**
     * @brief Update event 
     */
    virtual void Update(const gazebo::ommon::UpdateInfo& _info);

protected:

    /**
     * @brief Update event Cb 
     */
    virtual void Connect();

    /**
     * @brief Update position of UUV base 
     */
    void UpdateBasePose(ConstPose3dPtr& _msg);

    ignition::math::Pose3d base_link_pose_;

    gazebo::event::ConnectionPtr updateConnection_;

    gazebo::physics::WorldPtr world_;

    gazebo::physics::ModelPtr model_;

    gazebo::transport::NodePtr node_;

    std::string baseLinkName_;

    gazebo::transport::SubscriberPtr poseSub_;

}; // BaseUpdatePlugin
}; // ns

#endif

