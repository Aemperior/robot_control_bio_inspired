#pragma once

#include "kinematics.h"
#include <functional>

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
    enum motor motor1,
    enum motor motor2,
    float initial_angle1,
    float initial_angle2,
};

class CurrentLoopController {
public:
    // using MotorWriteFunction = std::function<void(float, int)>;
    // using MotorReadCurrentFunction = std::function<uint32_t()>;
    // using EncoderVelocityFunction = std::function<float()>;

    QEI encoder1; 
    QEI encoder2;
    MotorShield motorshield; 
    struct leg_config leg_conf; 

    // CurrentLoopController(
    //                       float duty_max,
    //                       MotorWriteFunction motorWrite_1_Func, 
    //                       MotorReadCurrentFunction motorReadCurrent_1_Func,
    //                       EncoderVelocityFunction motorReadVelocity_1_Func,
    //                       MotorWriteFunction motorWrite_2_Func,
    //                       MotorReadCurrentFunction motorReadCurrent_2_Func,
    //                       EncoderVelocityFunction motorReadVelocity_2_Func);
    CurrentLoopController(
                            float duty_max,
                            struct leg_config leg_conf,
                            QEI encoder1,
                            QEI encoder2,
                            MotorShield motorshield
    )

    void setParameters(/* parameters */);

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

private:

    // MotorWriteFunction motorWrite_1_Func;
    // MotorReadCurrentFunction motorReadCurrent_1_Func;
    // EncoderVelocityFunction motorReadVelocity_1_Func;
    // MotorWriteFunction motorWrite_2_Func;
    // MotorReadCurrentFunction motorReadCurrent_2_Func;
    // EncoderVelocityFunction motorReadVelocity_2_Func;


    // Your parameters here

    struct leg_gain gains; 

    struct joint_state joint_states; 

    float duty_max;
    float current_Kp = 4.0f;         
    float current_Ki = 0.4f;           
    float current_int_max = 3.0f;   

    float current_int1 = 0.0f;
    float current_int2 = 0.0f;
};

float readCurrent(motor m); 

float readVelocity(motor m);

float readAngle(motor m, float initialAngle); 

void writeMotor(motor m, float dutyCycle, int direction);
