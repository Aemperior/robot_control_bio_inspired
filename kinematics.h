#pragma once

typedef struct{
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
} kinematic_params; 

typedef struct{
    float Jx_th1; 
    float Jx_th2; 
    float Jy_th1; 
    float Jy_th2; 
} foot_jacobian;

typedef struct{
float xFoot; 
float yFoot; 
float dxFoot; 
float dyFoot; 
} foot_state;

typedef struct{
float th1; 
float th2; 
float dth1; 
float dth2; 
} joint_state;

typedef struct{
    float th1;
    float th2;
} angle_pair; 

foot_jacobian calc_foot_jacobi(float th1, float th2, kinematic_params params); 

foot_state calc_forward_kinematics(joint_state joints, kinematic_params params); 

angle_pair calc_inverse_kinematics(float xFoot, float yFoot, kinematic_params params); 

joint_state calc_desired_joints(foot_state foot_state_desired, foot_jacobian J, kinematic_params params); 