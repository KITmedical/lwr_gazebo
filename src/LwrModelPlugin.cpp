#include <stdio.h>

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>

#include <ros/ros.h>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public:
      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
      {
        std::cout << "------------------- LwrModelPlugin -------------------" << std::endl;

        if (!ros::isInitialized()) {
          ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
          return;
        }

        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ModelPush::OnUpdate, this));
      }

      // Called by the world update start event
      void OnUpdate()
      {
      }

    private:
      // Pointer to the model
      physics::ModelPtr model;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
