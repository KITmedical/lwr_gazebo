#include "LwrModelFRIPlugin.h"

// system includes
#include <stdio.h>

// library includes
#include <boost/bind.hpp>

// custom includes
#include <lwr/LwrLibrary.hpp>


namespace gazebo
{
/*---------------------------------- public: -----------------------------{{{-*/
  void
  LwrModelFRIPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
  {
    std::cout << "------------------- LwrModelFRIPlugin -------------------" << std::endl;

    m_model = _parent;

    if (!loadParams(_sdf)) {
      std::cout << "Error during loadParams" << std::endl;
      return;
    }


    m_joints = m_model->GetJoints();
    std::cout << "LWR joints:" << std::endl;
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      physics::JointPtr currJoint = m_joints[jointIdx];
      std::cout << jointIdx << " name=" << currJoint->GetName() << " angle=" << currJoint->GetAngle(0) << " v=" << currJoint->GetVelocity(0) << std::endl;
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&LwrModelFRIPlugin::OnUpdate, this));
  }

  void
  LwrModelFRIPlugin::OnUpdate()
  {
    updateRobotState();
    publishRobotState();
  }
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
  bool
  LwrModelFRIPlugin::loadParams(sdf::ElementPtr _sdf)
  {

    return true;
  }

  void
  LwrModelFRIPlugin::updateRobotState()
  {
  }

  void
  LwrModelFRIPlugin::publishRobotState()
  {
  }
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LwrModelFRIPlugin)
}
