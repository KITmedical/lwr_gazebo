#include "LwrModelFRIPlugin.h"

// system includes
#include <stdio.h>

// library includes
#include <boost/bind.hpp>

// custom includes
#include <lwr/LwrLibrary.hpp>
#include <ahbstring.h>


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

    unsigned lwrNameLen = std::string("lwrN").size();
    std::string pluginName = _sdf->GetAttribute("name")->GetAsString();
    std::string lwrName = pluginName.substr(pluginName.size() - lwrNameLen);

#if 0
    std::cout << "_sdf: name=" << _sdf->GetName() << " attributeCount=" << _sdf->GetAttributeCount() << std::endl;
    std::cout << "Attributes: " << std::endl;
    for (unsigned attrIdx = 0; attrIdx < _sdf->GetAttributeCount(); attrIdx++) {
      sdf::ParamPtr param = _sdf->GetAttribute(attrIdx);
      std::cout << attrIdx << " key=" << param->GetKey() << " value=" << param->GetAsString() << std::endl;
    }

    std::cout << "LWR name: " << m_model->GetName() << " childCount=" << m_model->GetChildCount() << std::endl;
    std::cout << "Children: " << std::endl;
    for (unsigned childIdx = 0; childIdx < m_model->GetChildCount(); childIdx++) {
      physics::BasePtr child = m_model->GetChild(childIdx);
      std::cout << childIdx << " name=" << child->GetName();
      std::cout << std::endl;
    }
#endif 

    // Extract only joints belonging to current lwr, even if it is part of larger model
    physics::Joint_V joints = m_model->GetJoints();
    std::cout << lwrName << " joints:" << std::endl;
    for (size_t jointIdx = 0; jointIdx < joints.size(); jointIdx++) {
      physics::JointPtr currJoint = joints[jointIdx];
      if (lwrName == currJoint->GetName().substr(0, lwrNameLen)) {
        m_joints.push_back(currJoint);
        std::cout << jointIdx << " name=" << currJoint->GetName() << " angle=" << currJoint->GetAngle(0) << " v=" << currJoint->GetVelocity(0) << std::endl;
      }
    }

    // TODO
    // send tFriMsrData to m_friPort (i.e. what friremote receives)
    // listen on m_friPort for tFriCmdData (i.e. what friremote sends)


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
    m_friPort = ahb::string::toIntSlow<uint16_t>(_sdf->GetElement("friPort")->Get<std::string>());

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
