#pragma once

#include "kinematics.h"
#include "controller.h"

#define BEZIER_ORDER_FOOT    7
#define N_FOOT_POINTS_PER_FOOT 2*(BEZIER_ORDER_FOOT+1)
#define NUM_INPUTS (16 + 2*N_FOOT_POINTS_PER_FOOT)
#define NUM_OUTPUTS 37

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

#define angleL1_OUT_IDX 19
#define velocityL1_OUT_IDX 20
#define currentL1_OUT_IDX 21
#define current_desL1_OUT_IDX 22
#define duty_cycleL1_OUT_IDX 23

#define angleL2_OUT_IDX 24
#define velocityL2_OUT_IDX 25
#define currentL2_OUT_IDX 26
#define current_desL2_OUT_IDX 27
#define duty_cycleL2_OUT_IDX 28

#define xFootL_OUT_IDX 29
#define yFootL_OUT_IDX 30
#define dxFootL_OUT_IDX 31
#define dyFootL_OUT_IDX 32

#define rDesFootL_x_OUT_IDX 33
#define rDesFootL_y_OUT_IDX 34
#define vDesFootL_x_OUT_IDX 35
#define vDesFootL_y_OUT_IDX 36


#define start_period_IN_IDX 0
#define start_periodR_IN_IDX 1
#define start_periodL_IN_IDX 2

#define traj_period_IN_IDX 3
#define end_period_IN_IDX 4
#define angleR1_init_IN_IDX 5
#define angleR2_init_IN_IDX 6
#define angleL1_init_IN_IDX 7
#define angleL2_init_IN_IDX 8

#define K_xx_IN_IDX 9
#define K_yy_IN_IDX 10
#define K_xy_IN_IDX 11
#define D_xx_IN_IDX 12
#define D_yy_IN_IDX 13
#define D_xy_IN_IDX 14
#define duty_max_IN_IDX 15

#define p_trajR_x0_IN_IDX 16
#define p_trajR_y0_IN_IDX 17
#define p_trajR_rx_IN_IDX 18
#define p_trajR_ry_IN_IDX 19
#define p_trajR_omega_IN_IDX 20
#define p_trajR_phase_delay_IN_IDX 21

#define p_trajL_x0_IN_IDX 22
#define p_trajL_y0_IN_IDX 23
#define p_trajL_rx_IN_IDX 24
#define p_trajL_ry_IN_IDX 25
#define p_trajL_omega_IN_IDX 26
#define p_trajL_phase_delay_IN_IDX 27

#define traj_mode_IN_IDX 28

#define bezier_pointsR_begin_IN_IDX 29

#define bezier_pointsL_begin_IN_IDX 29 + N_FOOT_POINTS_PER_FOOT

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
    float dutycycleL1; 
    float dutycycleL2; 
};

enum traj_mode{
    TRAJ_MODE_BEZIER = 0,
    TRAJ_MODE_ELLIPSE = 1,
};

struct from_matlab{
    float start_period;
    float start_periodR; 
    float start_periodL;
    float traj_period; 
    float end_period;

    float angleR1_init; 
    float angleR2_init; 

    float angleL1_init;
    float angleL2_init;
    
    struct leg_gain gains; 

    float duty_max; 

    struct p_traj p_trajR;
    struct p_traj p_trajL;

    enum traj_mode traj_mode;

    float foot_pointsR[N_FOOT_POINTS_PER_FOOT]; 
    float foot_pointsL[N_FOOT_POINTS_PER_FOOT];
};

void output_struct2array(struct to_matlab output_struct, float output_data[NUM_OUTPUTS]); 

void input_array2struct(float input_params[NUM_INPUTS], struct from_matlab *input_struct_ptr);