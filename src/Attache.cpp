#include <attache/Attache.h>


namespace gazebo {
  void Attache::Load(physics::WorldPtr wpWorld, sdf::ElementPtr epPtr) {
    this->m_wpWorld = wpWorld;
    
    // TODO: Set up ROS services
    
    this->m_cpUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Attache::OnUpdate, this, _1));
  }
  
  void Attache::OnUpdate(const common::UpdateInfo &uiInfo) {
    // TODO: spinOnce
  }
}
