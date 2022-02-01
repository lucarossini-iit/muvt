#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include <chrono>

namespace gazebo
{
    class MoveBox : public ModelPlugin
    {
        public: void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/)
        {
            // Stode the pointer to the model
            this->model = parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MoveBox::OnUpdate, this));

            // Set iterator to initial value
            _index = 0;
            _incr = 1;

            // Set model initial position
            // Send a pre-computed trajectory
            double y = -2.5;
            double x = 0.976034;
            double z = 1.22682;

            ignition::math::Pose3d pose(ignition::math::Vector3d(x, y, z), ignition::math::Quaterniond::Identity);
            this->model->SetLinkWorldPose(pose, "box::link");
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            this->model->SetLinearVel(ignition::math::Vector3d(0.0, 0.1, 0.0));
        }

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        // Trajectory iterator
        private: int _index;
        private: int _incr;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MoveBox)
}
