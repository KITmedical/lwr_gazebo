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

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&LwrModelPlugin::OnUpdate, this));
  }

  void
  LwrModelPlugin::OnUpdate()
  {
    m_cartesianReadTopicPub.publish(m_cartesianPoseCurrent);
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

    return true;
  }

  void
  LwrModelPlugin::cartesianWriteCallback(const geometry_msgs::Pose::ConstPtr& pose)
  {
    std::cout << "cartesianWriteCallback" << std::endl;
  }
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LwrModelPlugin)
}
