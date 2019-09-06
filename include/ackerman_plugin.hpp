//
// Created by yossi on 9/6/19.
//

#pragma once

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>


namespace gazebo
{
    class ackerman_plugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        // Called by the world update start event
    public:
        void OnUpdate();

        // Pointer to the model
    private:
        physics::ModelPtr model;
    private:
        physics::JointPtr steering_joint;
    private:
        physics::JointPtr leftW_joint;
    private:
        physics::JointPtr rightW_joint;
        physics::LinkPtr RL_wheel;
        physics::LinkPtr FL_wheel;
        physics::LinkPtr RR_wheel;

        double max_steering_angle = M_PI * 30 / 180;
        double steering_angle_factor;
        double L; // Longitudal separation between wheels
        double W; // Laterla separation between wheels

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ackerman_plugin)
}
