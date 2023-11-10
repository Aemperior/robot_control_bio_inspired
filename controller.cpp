#include "controller.h"

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

struct current_pair get_desired_current(struct joint_state state, struct leg_gain gains, struct joint_state desired_state, float k_t){
    
    float current_des1 = (gains.K_xx*(desired_state.th1 - state.th1) + gains.D_xx*(desired_state.dth1 - state.dth1))/k_t; 
    float current_des2 = (gains.K_yy*(desired_state.th2 - state.th2) + gains.D_yy*(desired_state.dth2 - state.dth2))/k_t;  

    struct current_pair desired_current_pair = {
        .current1 = current_des1,
        .current2 = current_des2,
    };

    return desired_current_pair; 
}


CurrentLoopController::CurrentLoopController(
                          float duty_max,
                          MotorWriteFunction motorWrite_1_Func, 
                          MotorReadCurrentFunction motorReadCurrent_1_Func,
                          EncoderVelocityFunction motorReadVelocity_1_Func,
                          MotorWriteFunction motorWrite_2_Func,
                          MotorReadCurrentFunction motorReadCurrent_2_Func,
                          EncoderVelocityFunction motorReadVelocity_2_Func) :    
    motorWrite_1_Func(motorWrite_1_Func),
    motorReadCurrent_1_Func(motorReadCurrent_1_Func),
    motorReadVelocity_1_Func(motorReadVelocity_1_Func),
    motorWrite_2_Func(motorWrite_2_Func),
    motorReadCurrent_2_Func(motorReadCurrent_2_Func),
    motorReadVelocity_2_Func(motorReadVelocity_2_Func)
{

    // Your parameters here 
    this->duty_max = duty_max;
}

void CurrentLoopController::callback()
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    // current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    // velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity   
    currents.current1 = -(((float(motorReadCurrent_1_Func)/65536.0f)*30.0f)-15.0f);           // measure current
    joint_states.dth1 = motorReadVelocity_1_Func * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1 = desired_currents.current1 - currents.current1;                                             // current errror
    current_int1 += err_c1;                                                             // integrate error
    current_int1 = fmaxf( fminf(current_int1, current_int_max), -current_int_max);      // anti-windup
    float ff1 = R*desired_currents.current1 + k_t*joint_states.dth1;                                         // feedforward terms
    duty_cycle1 = (ff1 + current_Kp*err_c1 + current_Ki*current_int1)/supply_voltage;   // PI current controller
    
    float absDuty1 = abs(duty_cycle1);
    
    if (absDuty1 > duty_max) {
        duty_cycle1 *= duty_max / absDuty1;
        absDuty1 = duty_max;
    }    
    if (duty_cycle1 < 0) { // backwards
        motorWrite_1_Func(absDuty1, 1);
    } else { // forwards
        motorWrite_1_Func(absDuty1, 0);
    }             
    

    currents.current2 = -(((float(motorReadCurrent_1_Func)/65536.0f)*30.0f)-15.0f);       // measure current
    joint_states.dth2 = motorReadVelocity_2_Func * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = desired_currents.current2 - currents.current2;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max), -current_int_max);      // anti-windup   
    float ff2 = R*desired_currents.current2 + k_t*joint_states.dth2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycleR2);
    if (absDuty2 > duty_max) {
        duty_cycle2 *= duty_max / absDuty2;
        absDuty2 = duty_max;
    }    
    if (duty_cycle2 < 0) { // backwards
        motorWrite_2_Func(absDuty2, 1);
    } else { // forwards
        motorWrite_2_Func(absDuty2, 0);
    }   

    previous_currents = currents;           
    
}