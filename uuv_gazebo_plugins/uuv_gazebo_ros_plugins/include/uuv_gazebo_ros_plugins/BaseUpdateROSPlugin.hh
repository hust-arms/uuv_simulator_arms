/*                                                                                             
 * Filename: uuv_gazebo_plugins/BodyUpdateROSPlugin.hh
 * Path: uuv_gazebo_plugins
 * Created Date: Friday, Janurary 29th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef __BASE_UPDATE_ROS_PLUGIN_HH__                                                    
#define __BASE_UPDATE_ROS_PLUGIN_HH__

#include <uuv_gazebo_plugins/BaseUpdatePlugin.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <uuv_gazebo_ros_plugins_msgs/GetModelProperties.h>
#include <uuv_gazebo_ros_plugins_msgs/SetFloat.h>
#include <uuv_gazebo_ros_plugins_msgs/GetFloat.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace uuv_simulator_ros
{
class BaseUpdateROSPlugin : public gazebo::BaseUpdatePlugin
{
public:
    /**
     * @brief Gazebo plugin ros wrapper class for UUV base status update
     */
    BaseUpdateROSPlugin();

    /**
     * @brief Destructor
     */
    virtual ~BaseUpdateROSPlugin();

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
     * @brief Reset module
     */
    virtual void Reset();

    /**
     * @brief Update event
     */
    virtual void Update(const gazebo::common::UpdateInfo& _info);

protected:
    /**
     * @brief Update position of UUV base 
     */                                                                                        
    void UpdateBasePose(const geometry_msgs::Pose::ConstPtr& _msg);

private:
    boost::scoped_ptr<ros::NodeHandle> rosNode_;

    ros::Subscriber baseLinkStatesSub_;

    geometry_msgs::TransformStamped nedTransform_;

    tf2_ros::TransformBroadcaster tfBroadcaster_;

}; // BaseUpdateROSPlugin
}; // ns

#endif

