#ifndef _LWR_MODEL_PLUGIN_H_
#define _LWR_MODEL_PLUGIN_H_

// system includes

// library includes
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// custom includes


// forward declarations


/* TODO
 * move to FRI interface
 * - send fri messages with 1000 Hz SIMULATED time (http://wiki.ros.org/Clock#Using_Simulation_Time_from_the_.2BAC8-clock_Topic)
 * - read answer messages and set m_joints
 */

namespace gazebo {
  class LwrModelFRIPlugin
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
      void updateRobotState();
      void publishRobotState();
  
      // variables
      // gazebo
      physics::ModelPtr m_model;
      event::ConnectionPtr m_updateConnection;
      physics::Joint_V m_joints;

      // ros
  
  
  };
}

#endif // _LWR_MODEL_PLUGIN_H_
