#include "controller.h"

struct current_pair get_desired_current(struct joint_state state, struct leg_gain gains, struct joint_state desired_state, float k_t){
    
    float current_des1 = (gains.K_xx*(desired_state.th1 - state.th1) + gains.D_xx*(desired_state.dth1 - state.dth1))/k_t; 
    float current_des2 = (gains.K_yy*(desired_state.th2 - state.th2) + gains.D_yy*(desired_state.dth2 - state.dth2))/k_t;  

    struct current_pair desired_current_pair = {
        .current1 = current_des1,
        .current2 = current_des2,
    };

    return desired_current_pair; 
}



void CurrentLoop(float *current1_ptr, float *current2_ptr)
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    // current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    // velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity   
    *current1_ptr = -(((float(motorShield.readCurrentD())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity1 = encoderD.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1 = current_des1 - *current1_ptr;                                             // current errror
    current_int1 += err_c1;                                                             // integrate error
    current_int1 = fmaxf( fminf(current_int1, current_int_max), -current_int_max);      // anti-windup
    float ff1 = R*current_des1 + k_t*velocity1;                                         // feedforward terms
    duty_cycleR1 = (ff1 + current_Kp*err_c1 + current_Ki*current_int1)/supply_voltage;   // PI current controller
    
    float absDuty1 = abs(duty_cycleR1);
    
    if (absDuty1 > duty_max) {
        duty_cycleR1 *= duty_max / absDuty1;
        absDuty1 = duty_max;
    }    
    if (duty_cycleR1 < 0) { // backwards
        motorShield.motorAWrite(absDuty1, 1);
        motorShield.motorDWrite(absDuty1, 1);
    } else { // forwards
        motorShield.motorAWrite(absDuty1, 0);
        motorShield.motorDWrite(absDuty1, 0);
    }             
    prev_current_des1 = current_des1; 
    
    // current2     = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f)-15.0f);       // measure current
    // velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity 
    *current2_ptr     = -(((float(motorShield.readCurrentC())/65536.0f)*30.0f)-15.0f);       // measure current
    velocity2 = encoderC.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = current_des2 - *current2_ptr;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max), -current_int_max);      // anti-windup   
    float ff2 = R*current_des2 + k_t*velocity2;                                         // feedforward terms
    duty_cycleR2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycleR2);
    if (absDuty2 > duty_max) {
        duty_cycleR2 *= duty_max / absDuty2;
        absDuty2 = duty_max;
    }    
    if (duty_cycleR2 < 0) { // backwards
        motorShield.motorBWrite(absDuty2, 1);
        motorShield.motorCWrite(absDuty2, 1);
    } else { // forwards
        motorShield.motorBWrite(absDuty2, 0);
        motorShield.motorCWrite(absDuty2, 0);
    }             
    prev_current_des2 = current_des2; 
    
}