#include <attache/Attache.h>


namespace gazebo {
  Attache::Attache() : m_nhHandle("~") {
  }
  
  Attache::~Attache() {
  }
  
  void Attache::Load(physics::WorldPtr wpWorld, sdf::ElementPtr epPtr) {
    this->m_wpWorld = wpWorld;
    
    m_srvAttach = m_nhHandle.advertiseService<Attache>("attach", &Attache::serviceAttach, this);
    m_srvDetach = m_nhHandle.advertiseService<Attache>("detach", &Attache::serviceDetach, this);
    
    this->m_cpUpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Attache::OnUpdate, this, _1));
    
    std::cout << this->title() << " Attache plugin loaded" << std::endl;
  }
  
  void Attache::OnUpdate(const common::UpdateInfo &uiInfo) {
    ros::spinOnce();
  }
  
  bool Attache::deleteJointIfPresent(std::string strLink1, std::string strLink2) {
    bool bReturnvalue = false;
    
    if(m_mapJoints.find(strLink1) != m_mapJoints.end()) {
      if(m_mapJoints[strLink1].find(strLink2) != m_mapJoints[strLink1].end()) {
	physics::JointPtr jpJoint = m_mapJoints[strLink1][strLink2];
	
	jpJoint->Detach();
	jpJoint->Update();
	jpJoint->Reset();
	jpJoint->Fini();
	
	m_mapJoints[strLink1].erase(strLink2);
	
	if(m_mapJoints[strLink1].size() == 0) {
	  m_mapJoints.erase(strLink1);
	}
	
	bReturnvalue = true;
      }
    }
    
    return bReturnvalue;
  }
  
  bool Attache::serviceAttach(attache_msgs::Attachment::Request &req, attache_msgs::Attachment::Response &res) {
    std::string strLink1 = req.model1 + "." + req.link1;
    std::string strLink2 = req.model2 + "." + req.link2;
    
    gazebo::physics::ModelPtr mpLink1 = this->m_wpWorld->GetModel(req.model1);
    
    if(mpLink1) {
      const physics::LinkPtr lpLink1 = mpLink1->GetLink(req.link1);
      
      if(lpLink1) {
	gazebo::physics::ModelPtr mpLink2 = this->m_wpWorld->GetModel(req.model2);
	
	if(mpLink2) {
	  const physics::LinkPtr lpLink2 = mpLink2->GetLink(req.link2);
	  
	  if(lpLink2) {
	    this->deleteJointIfPresent(strLink1, strLink2);
	    this->deleteJointIfPresent(strLink2, strLink1);
	    
	    std::cout << this->title() << " Attach link '" << strLink2 << "' to link '" << strLink1 << "'" << std::endl;
	    
	    m_mapJoints[strLink1][strLink2] = this->m_wpWorld->GetPhysicsEngine()->CreateJoint("revolute", mpLink1);
	    m_mapJoints[strLink1][strLink2]->Load(lpLink1, lpLink2, math::Pose());
	    
	    m_mapJoints[strLink1][strLink2]->Init();
	    m_mapJoints[strLink1][strLink2]->SetHighStop(0, 0);
	    m_mapJoints[strLink1][strLink2]->SetLowStop(0, 0);
	    
	    res.success = true;
	  } else {
	    std::cerr << this->title(true) << " No link '" << req.model2 << "." << req.link2 << "'" << std::endl;
	  }
	} else {
	  std::cerr << this->title(true) << " No model '" << req.model2 << "'" << std::endl;
	}
      } else {
	std::cerr << this->title(true) << " No link '" << req.model1 << "." << req.link1 << "'" << std::endl;
      }
    } else {
      std::cerr << this->title(true) << " No model '" << req.model1 << "'" << std::endl;
    }
    
    return true;
  }
  
  bool Attache::serviceDetach(attache_msgs::Attachment::Request &req, attache_msgs::Attachment::Response &res) {
    std::string strLink1 = req.model1 + "." + req.link1;
    std::string strLink2 = req.model2 + "." + req.link2;
    
    if(this->deleteJointIfPresent(strLink1, strLink2) || this->deleteJointIfPresent(strLink2, strLink1)) {
      std::cout << this->title() << " Detached link '" << strLink2 << "' from link '" << strLink1 << "'" << std::endl;
      
      res.success = true;
    } else {
      std::cerr << this->title(true) << " No connection to detach between '" << strLink1 << "' and '" << strLink2 << "'" << std::endl;
      
      res.success = false;
    }
    
    return true;
  }
  
  std::string Attache::title(bool bFailed) {
    if(bFailed) {
      return "\033[1;31m[Attache]\033[0m";
    } else {
      return "\033[1;37m[Attache]\033[0m";
    }
  }
}
