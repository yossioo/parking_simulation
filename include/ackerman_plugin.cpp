//
// Created by yossi on 9/6/19.
//

#include "ackerman_plugin.hpp"


using namespace gazebo;

void ackerman_plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;
    this->steering_joint = model->GetJoint("steering_joint");
    this->leftW_joint = model->GetJoint("front_left_steering_joint");
    this->rightW_joint = model->GetJoint("front_right_steering_joint");
    FL_wheel = model->GetLink("front_left_wheel");
    RL_wheel = model->GetLink("rear_left_wheel");
    RR_wheel = model->GetLink("rear_right_wheel");

    auto FL_pose = FL_wheel->WorldCoGPose();
    auto RL_pose = RL_wheel->WorldCoGPose();
    auto RR_pose = RR_wheel->WorldCoGPose();
    auto length = FL_pose.CoordPositionSub(RL_pose);
    auto width = RR_pose.CoordPositionSub(RL_pose);
    this->L = length.Length();
    this->W = width.Length();

    printf("L=%2.2f\tW=%2.2f\n", L, W);

    steering_angle_factor = max_steering_angle / steering_joint->UpperLimit();
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ackerman_plugin::OnUpdate, this));
}

void ackerman_plugin::OnUpdate()
{
    auto steering_angle = steering_joint->Position() * steering_angle_factor;

    double ca = cos(steering_angle);
    double sa = sin(steering_angle);

    double ai = atan2(2 * L * sa, 2 * L * ca - W * sa);
    double ao = atan2(2 * L * sa, 2 * L * ca + W * sa);
    leftW_joint->SetPosition(0, ai);
    rightW_joint->SetPosition(0, ao);
}
