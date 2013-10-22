#include "LwrModelROSPlugin.h"

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
  LwrModelROSPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
  {
    std::cout << "------------------- LwrModelROSPlugin -------------------" << std::endl;

    m_model = _parent;

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("LwrModelROSPlugin: A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    if (!loadParams(_sdf)) {
      ROS_FATAL_STREAM("Error during loadParams");
      return;
    }


    m_node = new ros::NodeHandle(m_nodeName);
    m_cartesianWriteTopicSub = m_node->subscribe<geometry_msgs::Pose>(m_cartesianWriteTopicName, 1, &LwrModelROSPlugin::cartesianWriteCallback, this);
    m_cartesianReadTopicPub = m_node->advertise<geometry_msgs::Pose>(m_cartesianReadTopicName, 1);
    m_jointsWriteTopicSub = m_node->subscribe<sensor_msgs::JointState>(m_jointsWriteTopicName, 1, &LwrModelROSPlugin::jointsWriteCallback, this);
    m_jointsReadTopicPub = m_node->advertise<sensor_msgs::JointState>(m_jointsReadTopicName, 1);

    unsigned lwrNameLen = std::string("lwrN").size();
    std::string pluginName = _sdf->GetAttribute("name")->GetAsString();
    std::string lwrName = pluginName.substr(pluginName.size() - lwrNameLen);

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

    m_jointsCurrent.position.resize(m_joints.size(), 0);
    m_jointsCurrent.velocity.resize(m_joints.size(), 0);
    m_jointsCurrent.effort.resize(m_joints.size(), 0);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&LwrModelROSPlugin::OnUpdate, this));
  }

  void
  LwrModelROSPlugin::OnUpdate()
  {
    updateRobotState();
    publishRobotState();
  }
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
  bool
  LwrModelROSPlugin::loadParams(sdf::ElementPtr _sdf)
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
  LwrModelROSPlugin::cartesianWriteCallback(const geometry_msgs::Pose::ConstPtr& poseMsg)
  {
    //std::cout << "cartesianWriteCallback: poseMsg=" << *poseMsg << std::endl;
    tf::Pose tfpose;
    tf::poseMsgToTF(*poseMsg, tfpose);
    tf::Vector3 tforigin = tfpose.getOrigin();
    tf::Quaternion tforientation = tfpose.getRotation();
    LwrXCart cartXPose;
    cartXPose.nsparam = 0;
    cartXPose.config = 2;
    cartXPose.pose.setPos(tforigin.x(), tforigin.y(), tforigin.z());
    double x, y, z, w;
    w = tforientation.w();
    x = tforientation.x();
    y = tforientation.y();
    z = tforientation.z();
    cartXPose.pose.setQuat(w, x, y, z);
    LwrJoints joints;
    LwrErrorMsg kinematicReturn;
    kinematicReturn = Lwr::inverseKinematics(joints, cartXPose);
    if (kinematicReturn != LWR_OK && kinematicReturn != (LWR_WARNING | LWR_CLOSE_TO_SINGULARITY)) {
      ROS_WARN_STREAM("Lwr::inverseKinematics() failed: " << kinematicReturn);
      return;
    }
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      m_joints[jointIdx]->SetAngle(0, joints.j[jointIdx]);
    }
  }

  void
  LwrModelROSPlugin::jointsWriteCallback(const sensor_msgs::JointState::ConstPtr& jointsMsg)
  {
    //std::cout << "jointsWriteCallback: jointsMsg=" << *jointsMsg << std::endl;
    if (jointsMsg->position.size() != m_jointsCurrent.position.size()) {
      ROS_WARN("Wrong number of joints received. Ignoring message.");
      return;
    }
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      m_joints[jointIdx]->SetAngle(0, jointsMsg->position[jointIdx]);
    }
  }

  void
  LwrModelROSPlugin::updateRobotState()
  {
    // joints
    for (size_t jointIdx = 0; jointIdx < m_joints.size(); jointIdx++) {
      physics::JointPtr currJoint = m_joints[jointIdx];
      m_jointsCurrent.position[jointIdx] = currJoint->GetAngle(0).Radian();
      m_jointsCurrent.velocity[jointIdx] = currJoint->GetVelocity(0);
    }

    // cartesian
    LwrXCart cartXPose;
    LwrJoints joints;
    joints.setJoints(m_jointsCurrent.position);
    LwrErrorMsg kinematicReturn;
    kinematicReturn = Lwr::forwardKinematics(cartXPose, joints);
    if (kinematicReturn != LWR_OK && kinematicReturn != (LWR_WARNING | LWR_SINGULARITY)) {
      ROS_WARN_STREAM("Lwr::forwardKinematics() failed: " << kinematicReturn);
      return;
    }
    cartXPose.pose.getPos(m_cartesianPoseCurrent.position.x, m_cartesianPoseCurrent.position.y, m_cartesianPoseCurrent.position.z);
    cartXPose.pose.getQuat(m_cartesianPoseCurrent.orientation.w, m_cartesianPoseCurrent.orientation.x, m_cartesianPoseCurrent.orientation.y, m_cartesianPoseCurrent.orientation.z);
  }

  void
  LwrModelROSPlugin::publishRobotState()
  {
    m_cartesianReadTopicPub.publish(m_cartesianPoseCurrent);
    m_jointsReadTopicPub.publish(m_jointsCurrent);
  }
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LwrModelROSPlugin)
}
