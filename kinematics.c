#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"
#include "Matrix.h"
#include "MatrixMath.h"
#include "kinematics.h"

foot_jacobian calc_foot_jacobi(float th1, float th2, kinematic_params params){
    float l_AC = params.l_AC; 
    float l_DE = params.l_DE; 
    float l_OB = params.l_OB; 
    
    float Jx_th1 = l_AC*cos(th1 + th2) + l_DE*cos(th1) + l_OB*cos(th1); //assumed to be dx/dth1
    float Jx_th2 = l_AC*cos(th1 + th2); //assumed to be dx/dth2
    float Jy_th1 = l_AC*sin(th1 + th2) + l_DE*sin(th1) + l_OB*sin(th1); //assumed to be dy/dth1
    float Jy_th2 = l_AC*sin(th1 + th2); //assumed to be dy/dth2

    foot_jacobian J = {
        .Jx_th1 = Jx_th1,
        .Jx_th2 = Jx_th2,
        .Jy_th1 = Jy_th1,
        .Jy_th2 = Jy_th2,
    }; 

    return J; 
}

foot_state calc_forward_kinematics(joint_state joints, kinematic_params params){
    float l_AC = params.l_AC; 
    float l_DE = params.l_DE; 
    float l_OB = params.l_OB; 

    float th1 = joints.th1; 
    float th2 = joints.th2; 
    float dth1 = joints.dth1; 
    float dth2 = joints.dth2; 
    
    float xFoot = l_AC*sin(th1+th2) + l_DE*sin(th1) + l_OB*sin(th1);
    float yFoot = -l_AC*cos(th1+th2) - l_DE*cos(th1) - l_OB*cos(th1);
    float dxFoot = dth1*(l_AC*cos(th1 + th2) + l_DE*cos(th1) + l_OB*cos(th1)) + dth2*l_AC*cos(th1 + th2);
    float dyFoot = dth1*(l_AC*sin(th1 + th2) + l_DE*sin(th1) + l_OB*sin(th1)) + dth2*l_AC*sin(th1 + th2);  

    foot_state state = {
        .xFoot = xFoot, 
        .yFoot = xFoot, 
        .dxFoot = dxFoot, 
        .dyFoot = dyFoot, 
    }; 

    return state;
}

angle_pair calc_inverse_kinematics(float xFoot, float yFoot, kinematic_params params){
    
    float l_AC = params.l_AC; 
    float l_DE = params.l_DE; 
    float l_OB = params.l_OB; 
    
    float l_OE = sqrt( (pow(xFoot,2) + pow(yFoot,2)) );
    float alpha = abs(acos( (pow(l_OE,2) - pow(l_AC,2) - pow((l_OB+l_DE),2))/(-2.0f*l_AC*(l_OB+l_DE)) ));
    float th2 = -(3.14159f - alpha); 
    float th1 = -((3.14159f/2.0f) + atan2(yFoot,xFoot) - abs(asin( (l_AC/l_OE)*sin(alpha) )));

    angle_pair angles = {
        .th1 = th1,
        .th2 = th2,
    };

    return angles; 
}

joint_state calc_desired_joints(foot_state foot_state_desired, foot_jacobian J, kinematic_params params){

    float rDesFoot_x = foot_state_desired.xFoot;
    float rDesFoot_y = foot_state_desired.yFoot;
    float vDesFoot_x = foot_state_desired.dxFoot; 
    float vDesFoot_y = foot_state_desired.dyFoot; 

    // Don't why minus is added, original code also did that
    float xFoot_inv = -rDesFoot_x;
    float yFoot_inv = rDesFoot_y;   

    angle_pair desired_angles = calc_inverse_kinematics(xFoot_inv,
                                                        yFoot_inv, 
                                                        params);

    float Jx_th1 = J.Jx_th1;
    float Jx_th2 = J.Jx_th2; 
    float Jy_th1 = J.Jy_th1;
    float Jy_th2 = J.Jy_th2; 

    float dd = (Jx_th1*Jy_th2 - Jx_th2*Jy_th1);
    float dth1_des = (1.0f/dd) * (  Jy_th2*vDesFoot_x - Jx_th2*vDesFoot_y );
    float dth2_des = (1.0f/dd) * ( -Jy_th1*vDesFoot_x + Jx_th1*vDesFoot_y );

    joint_state desired_angle_state = {
        .th1 = desired_angles.th1,        
        .th2 = desired_angles.th2,        
        .dth1 = dth1_des,       
        .dth2 = dth2_des,           
    }; 
}