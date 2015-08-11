#ifndef __ATTACHE_H__
#define __ATTACHE_H__


// System
#include <stdio.h>

// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


namespace gazebo {
  class Attache : public WorldPlugin {
  private:
    physics::WorldPtr m_wpWorld;
    event::ConnectionPtr m_cpUpdateConnection;
    
  protected:
  public:
    Attache();
    ~Attache();
    
    void Load(physics::WorldPtr wpWorld, sdf::ElementPtr epPtr);
    void OnUpdate(const common::UpdateInfo &uiInfo);
  };
  
  GZ_REGISTER_WORLD_PLUGIN(Attache)
}


#endif /* __ATTACHE_H__ */
