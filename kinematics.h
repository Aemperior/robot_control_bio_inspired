#pragma once

#include "BezierCurve.h"

struct kinematic_params{
    // Fixed kinematic parameters
    float l_OA;  
    float l_OB;  
    float l_AC;  
    float l_DE; 
    float m1 ;
    float m2 ;  
    float m3 ; 
    float m4 ; 
    float I1 ;
    float I2 ;
    float I3 ; 
    float I4 ;
    float l_O_m1;
    float l_B_m2; 
    float l_A_m3;
    float l_C_m4;
    float N;
    float Ir;
}; 

struct foot_jacobian{
    float Jx_th1; 
    float Jx_th2; 
    float Jy_th1; 
    float Jy_th2; 
};

struct foot_state{
float xFoot; 
float yFoot; 
float dxFoot; 
float dyFoot; 
};

struct joint_state{
float th1; 
float th2; 
float dth1; 
float dth2; 
};

struct angle_pair{
    float th1;
    float th2;
}; 

struct p_traj{
    float x0; 
    float y0; 
    float rx; 
    float ry; 
    float omega; 
    float phase_delay; 
};

struct foot_jacobian calc_foot_jacobi(float th1, float th2, struct kinematic_params params); 

struct foot_state calc_forward_kinematics(struct joint_state joints, struct kinematic_params params); 

struct angle_pair calc_inverse_kinematics(float xFoot, float yFoot, struct kinematic_params params); 

struct joint_state calc_desired_joints(struct foot_state foot_state_desired, struct foot_jacobian J, struct kinematic_params params);

struct foot_state calc_desired_foot_single_bezier(BezierCurve* desired_curve_ptr, float vMult, float t_eff, float traj_period); 

struct foot_state calc_desired_foot_ellipse(float teff, struct p_traj ellipse_traj); 
