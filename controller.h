#pragma once

#include "kinematics.h"

struct current_pair{
    float current1; 
    float current2; 
}; 

struct leg_gain{
    float K_xx;
    float K_yy;
    float K_xy;
    float D_xx;
    float D_xy;
    float D_yy;
};

// struct current_pair get_desired_current(struct joint_state state, struct leg_gain, struct joint_state desired_state); 