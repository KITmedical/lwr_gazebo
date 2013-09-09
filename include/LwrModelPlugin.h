#ifndef _LWR_MODEL_PLUGIN_H_
#define _LWR_MODEL_PLUGIN_H_

// system includes

// library includes
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// custom includes


// forward declarations


namespace gazebo {
  class LwrModelPlugin
    : public ModelPlugin
  {
    public:
      // enums
  
      // typedefs
  
      // const static member variables
   
      // static utility functions
  
  
      // constructors
  
      // overwritten methods
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
  
      // methods
      void OnUpdate();
  
      // variables
  
  
    private:
      // methods
      bool loadParams(sdf::ElementPtr _sdf);
      void cartesianWriteCallback(const geometry_msgs::Pose::ConstPtr& pose);
      void jointsWriteCallback(const sensor_msgs::JointState::ConstPtr& joints);
      void updateRobotState();
      void publishRobotState();
  
      // variables
      // gazebo
      physics::ModelPtr m_model;
      event::ConnectionPtr m_updateConnection;
      physics::Joint_V m_joints;

      // ros
      ros::NodeHandle* m_node;
      ros::Subscriber m_cartesianWriteTopicSub;
      ros::Publisher m_cartesianReadTopicPub;
      ros::Subscriber m_jointsWriteTopicSub;
      ros::Publisher m_jointsReadTopicPub;
      std::string m_nodeName;
      std::string m_robotNamespaceName;
      std::string m_cartesianReadTopicName;
      std::string m_cartesianWriteTopicName;
      std::string m_jointsReadTopicName;
      std::string m_jointsWriteTopicName;
      geometry_msgs::Pose m_cartesianPoseCurrent;
      sensor_msgs::JointState m_jointsCurrent;
  
  
  };
}

#endif // _LWR_MODEL_PLUGIN_H_
