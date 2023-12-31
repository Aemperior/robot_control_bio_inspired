#pragma once

#include "pragma_replacement.h"
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

struct current_pair get_desired_current(struct joint_state state, struct leg_gain, struct joint_state desired_state, float k_t); 

enum motor{
    MOTOR_A,
    MOTOR_B,
    MOTOR_C,
    MOTOR_D,    
};

struct leg_config{
    enum motor motor1;
    enum motor motor2;
};

class CurrentLoopController {
public:

    QEI *encoder1_ptr; 
    QEI *encoder2_ptr;
    MotorShield *motorshield_ptr; 
    struct leg_config leg_conf; 

    CurrentLoopController(
                            float duty_max,
                            struct leg_config leg_conf,
                            QEI *encoder1_ptr,
                            QEI *encoder2_ptr,
                            MotorShield *motorshield_ptr
    );

    void callback();

    struct current_pair currents; 
    struct current_pair desired_currents;
    struct current_pair previous_currents; 

    float duty_cycle1; 
    float duty_cycle2; 

    // Model parameters
    float supply_voltage = 12;     // motor supply voltage
    float R = 2.0f;                // motor resistance
    float k_t = 0.18f;             // motor torque constant
    float nu = 0.0005;             // motor viscous friction  

    float readCurrent(motor m); 
    
    void writeMotor(motor m, float dutyCycle, int direction);

private:

    struct leg_gain gains; 

    struct joint_state joint_states; 

    float duty_max;
    float current_Kp = 4.0f;         
    float current_Ki = 0.4f;           
    float current_int_max = 3.0f;   

    float current_int1 = 0.0f;
    float current_int2 = 0.0f;
};


