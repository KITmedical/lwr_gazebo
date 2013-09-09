#ifndef _LWR_MODEL_PLUGIN_H_
#define _LWR_MODEL_PLUGIN_H_

// system includes

// library includes
#include <gazebo/gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>

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
  
      // variables
      physics::ModelPtr m_model;
      event::ConnectionPtr m_updateConnection;
  
  
  };
}

#endif // _LWR_MODEL_PLUGIN_H_
