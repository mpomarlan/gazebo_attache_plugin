#ifndef __ATTACHE_H__
#define __ATTACHE_H__


// System
#include <stdio.h>
#include <iostream>
#include <map>
#include <string>

// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ROS
#include <ros/ros.h>
#include <tf/tfMessage.h>

// Attachment Messages
#include <attache_msgs/Attachment.h>


namespace gazebo {
  class Attache : public WorldPlugin {
  private:
    std::map< std::string, std::map<std::string, physics::JointPtr> > m_mapJoints; 
    
    ros::NodeHandle m_nhHandle;
    ros::ServiceServer m_srvAttach;
    ros::ServiceServer m_srvDetach;
    
    physics::WorldPtr m_wpWorld;
    event::ConnectionPtr m_cpUpdateConnection;
    
  protected:
  public:
    Attache();
    ~Attache();
    
    void Load(physics::WorldPtr wpWorld, sdf::ElementPtr epPtr);
    void OnUpdate(const common::UpdateInfo &uiInfo);
    
    bool deleteJointIfPresent(std::string strLink1, std::string strLink2);
    
    bool serviceAttach(attache_msgs::Attachment::Request &req, attache_msgs::Attachment::Response &res);
    bool serviceDetach(attache_msgs::Attachment::Request &req, attache_msgs::Attachment::Response &res);
    
    std::string title(bool bFailed = false);
  };
  
  GZ_REGISTER_WORLD_PLUGIN(Attache)
}


#endif /* __ATTACHE_H__ */
