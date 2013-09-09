#include "LwrModelPlugin.h"

// system includes
#include <stdio.h>

// library includes
#include <boost/bind.hpp>

// custom includes


namespace gazebo
{
/*---------------------------------- public: -----------------------------{{{-*/
  void
  LwrModelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
  {
    std::cout << "------------------- LwrModelPlugin -------------------" << std::endl;

    m_model = _parent;

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    if (!loadParams(_sdf)) {
      ROS_FATAL_STREAM("Error during loadParams");
      return;
    }


    m_node = new ros::NodeHandle(m_nodeName);
    m_cartesianWriteTopicSub = m_node->subscribe<geometry_msgs::Pose>(m_cartesianWriteTopicName, 1, &LwrModelPlugin::cartesianWriteCallback, this);
    m_cartesianReadTopicPub = m_node->advertise<geometry_msgs::Pose>(m_cartesianReadTopicName, 1);
    m_jointsWriteTopicSub = m_node->subscribe<sensor_msgs::JointState>(m_jointsWriteTopicName, 1, &LwrModelPlugin::jointsWriteCallback, this);
    m_jointsReadTopicPub = m_node->advertise<sensor_msgs::JointState>(m_jointsReadTopicName, 1);

    m_joints = m_model->GetJoints();
    std::cout << "LWR joints:" << std::endl;
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      physics::JointPtr currJoint = m_joints[jointIdx];
      std::cout << jointIdx << " name=" << currJoint->GetName() << " angle=" << currJoint->GetAngle(0) << " v=" << currJoint->GetVelocity(0) << std::endl;
    }

    m_jointsCurrent.position.resize(m_joints.size(), 0);
    m_jointsCurrent.velocity.resize(m_joints.size(), 0);
    m_jointsCurrent.effort.resize(m_joints.size(), 0);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&LwrModelPlugin::OnUpdate, this));
  }

  void
  LwrModelPlugin::OnUpdate()
  {
    updateRobotState();
    publishRobotState();
  }
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
  bool
  LwrModelPlugin::loadParams(sdf::ElementPtr _sdf)
  {
    m_nodeName = _sdf->GetParent()->Get<std::string>("name");

    m_robotNamespaceName = _sdf->GetElement("robotNamespace")->Get<std::string>();
    m_cartesianReadTopicName = m_robotNamespaceName + "/" + _sdf->GetElement("cartesianReadTopic")->Get<std::string>();
    m_cartesianWriteTopicName = m_robotNamespaceName + "/" + _sdf->GetElement("cartesianWriteTopic")->Get<std::string>();
    m_jointsReadTopicName = m_robotNamespaceName + "/" + _sdf->GetElement("jointsReadTopic")->Get<std::string>();
    m_jointsWriteTopicName = m_robotNamespaceName + "/" + _sdf->GetElement("jointsWriteTopic")->Get<std::string>();

    return true;
  }

  void
  LwrModelPlugin::cartesianWriteCallback(const geometry_msgs::Pose::ConstPtr& pose)
  {
    std::cout << "cartesianWriteCallback" << std::endl;
  }

  void
  LwrModelPlugin::jointsWriteCallback(const sensor_msgs::JointState::ConstPtr& joints)
  {
    if (joints->position.size() != m_jointsCurrent.position.size()) {
      ROS_WARN("Wrong number of joints received. Ignoring message.");
      return;
    }
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      m_joints[jointIdx]->SetAngle(0, joints->position[jointIdx]);
    }
  }

  void
  LwrModelPlugin::updateRobotState()
  {
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      physics::JointPtr currJoint = m_joints[jointIdx];
      m_jointsCurrent.position[jointIdx] = currJoint->GetAngle(0).Radian();
      m_jointsCurrent.velocity[jointIdx] = currJoint->GetVelocity(0);
    }
    // TODO m_cartesianPoseCurrent
  }

  void
  LwrModelPlugin::publishRobotState()
  {
    m_cartesianReadTopicPub.publish(m_cartesianPoseCurrent);
    m_jointsReadTopicPub.publish(m_jointsCurrent);
  }
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LwrModelPlugin)
}
