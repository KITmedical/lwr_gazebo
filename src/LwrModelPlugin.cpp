#include "LwrModelPlugin.h"

// system includes
#include <stdio.h>

// library includes
#include <boost/bind.hpp>

#include <ros/ros.h>

// custom includes


namespace gazebo
{
/*---------------------------------- public: -----------------------------{{{-*/
  void
  LwrModelPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
  {
    std::cout << "------------------- LwrModelPlugin -------------------" << std::endl;

    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Store the pointer to the model
    m_model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&LwrModelPlugin::OnUpdate, this));
  }

  // Called by the world update start event
  void
  LwrModelPlugin::OnUpdate()
  {
  }
/*------------------------------------------------------------------------}}}-*/

/*---------------------------------- private: ----------------------------{{{-*/
/*------------------------------------------------------------------------}}}-*/


  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(LwrModelPlugin)
}
