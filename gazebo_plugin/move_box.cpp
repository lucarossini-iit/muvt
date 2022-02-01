#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include <chrono>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 10.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0.0);
        key->Translation(ignition::math::Vector3d(0.976034, -0.1, 1.22682));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set first waypoint
        key = anim->CreateKeyFrame(3.0);
        key->Translation(ignition::math::Vector3d(0.976034, 0.3, 1.22682));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set second waypoint
        key = anim->CreateKeyFrame(5.0);
        key->Translation(ignition::math::Vector3d(0.976034, 0.1, 1.22682));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set third waypoint
        key = anim->CreateKeyFrame(7.0);
        key->Translation(ignition::math::Vector3d(0.946034, 0.1, 1.42682));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // move back to starting point
        key = anim->CreateKeyFrame(10.0);
        key->Translation(ignition::math::Vector3d(0.976034, -0.1, 1.22682));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}
