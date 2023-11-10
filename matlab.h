#pragma once

#include "kinematics.h"
#include "controller.h"

#define BEZIER_ORDER_FOOT    7
#define NUM_INPUTS (12 + 2*(BEZIER_ORDER_FOOT+1))
#define NUM_OUTPUTS 19
#define N_FOOT_POINTS 2*(BEZIER_ORDER_FOOT+1)

#define t_OUT_IDX 0

#define angleR1_OUT_IDX 1
#define velocityR1_OUT_IDX 2
#define currentR1_OUT_IDX 3
#define current_desR1_OUT_IDX 4
#define duty_cycleR1_OUT_IDX 5

#define angleR2_OUT_IDX 6
#define velocityR2_OUT_IDX 7
#define currentR2_OUT_IDX 8
#define current_desR2_OUT_IDX 9
#define duty_cycleR2_OUT_IDX 10

#define xFootR_OUT_IDX 11
#define yFootR_OUT_IDX 12
#define dxFootR_OUT_IDX 13
#define dyFootR_OUT_IDX 14

#define rDesFootR_x_OUT_IDX 15
#define rDesFootR_y_OUT_IDX 16
#define vDesFootR_x_OUT_IDX 17
#define vDesFootR_y_OUT_IDX 18


#define start_period_IN_IDX 0
#define traj_period_IN_IDX 1
#define end_period_IN_IDX 2
#define angleR1_init_IN_IDX 3
#define angleR2_init_IN_IDX 4
#define K_xx_IN_IDX 5
#define K_yy_IN_IDX 6
#define K_xy_IN_IDX 7
#define D_xx_IN_IDX 8
#define D_yy_IN_IDX 9
#define D_xy_IN_IDX 10
#define duty_max_IN_IDX 11
#define bezier_points_begin_IN_IDX 12

struct to_matlab{
    float t; 

    struct foot_state footR; 
    struct foot_state footL; 
    struct joint_state jointsR; 
    struct joint_state jointsL; 

    struct foot_state des_footR; 
    struct foot_state des_footL;
    struct joint_state des_jointsR; 
    struct joint_state des_jointsL; 

    struct current_pair current_pairR; 
    struct current_pair current_pairL; 

    struct current_pair des_current_pairR; 
    struct current_pair des_current_pairL; 

    float dutycycleR1; 
    float dutycycleR2; 
    float dutyCycleL1; 
    float dutyCycleL2; 
};

struct from_matlab{
    float start_period;
    float traj_period; 
    float end_period;

    float angleR1_init; 
    float angleR2_init; 
    
    struct leg_gain gains; 

    float duty_max; 

    float foot_points[N_FOOT_POINTS]; 
};

void output_struct2array(struct to_matlab output_struct, float output_data[NUM_OUTPUTS]); 

void input_array2struct(float input_params[NUM_INPUTS], struct from_matlab *input_struct_ptr);