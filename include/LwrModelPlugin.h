#ifndef _LWR_MODEL_PLUGIN_H_
#define _LWR_MODEL_PLUGIN_H_

// system includes

// library includes
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

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
  
      // variables
      // gazebo
      physics::ModelPtr m_model;
      event::ConnectionPtr m_updateConnection;

      // ros
      ros::NodeHandle* m_node;
      ros::Subscriber m_cartesianWriteTopicSub;
      ros::Publisher m_cartesianReadTopicPub;
      std::string m_nodeName;
      std::string m_robotNamespaceName;
      std::string m_cartesianReadTopicName;
      std::string m_cartesianWriteTopicName;
      geometry_msgs::Pose m_cartesianPoseCurrent;
  
  
  };
}

#endif // _LWR_MODEL_PLUGIN_H_
