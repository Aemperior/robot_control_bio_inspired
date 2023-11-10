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

struct current_pair get_desired_current(struct joint_state state, struct leg_gain, struct joint_state desired_state, float k_t); 

class CurrentLoopController {
public:
    using MotorWriteFunction = std::function<void(float, int)>;
    using MotorReadCurrentFunction = std::function<uint32_t()>;
    using EncoderVelocityFunction = std::function<float()>;

    CurrentLoopController(
                          float duty_max,
                          MotorWriteFunction motorWrite_1_Func, 
                          MotorReadCurrentFunction motorReadCurrent_1_Func,
                          EncoderVelocityFunction motorReadVelocity_1_Func,
                          MotorWriteFunction motorWrite_2_Func,
                          MotorReadCurrentFunction motorReadCurrent_2_Func,
                          EncoderVelocityFunction motorReadVelocity_2_Func);

    void setParameters(/* parameters */);

    void callback();

private:

    MotorWriteFunction motorWrite_1_Func;
    MotorReadCurrentFunction motorReadCurrent_1_Func;
    EncoderVelocityFunction motorReadVelocity_1_Func;
    MotorWriteFunction motorWrite_2_Func;
    MotorReadCurrentFunction motorReadCurrent_2_Func;
    EncoderVelocityFunction motorReadVelocity_2_Func;

    // Your parameters here
    struct current_pair currents; 
    struct current_pair desired_currents;
    struct current_pair previous_currents; 

    struct leg_gain gains; 

    struct joint_state joint_states; 

    float duty_max;
    float current_Kp = 4.0f;         
    float current_Ki = 0.4f;           
    float current_int_max = 3.0f;   

    float duty_cycle1; 
    float duty_cycle2;   

    // Model parameters
    float supply_voltage = 12;     // motor supply voltage
    float R = 2.0f;                // motor resistance
    float k_t = 0.18f;             // motor torque constant
    float nu = 0.0005;             // motor viscous friction

    float current_int1 = 0.0f;
    float current_int2 = 0.0f;
};