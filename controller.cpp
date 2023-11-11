#include "controller.h"

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

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
                            struct leg_config leg_conf,
                            QEI *encoder1_ptr,
                            QEI *encoder2_ptr,
                            MotorShield *motorshield_ptr) : 
    encoder1_ptr(encoder1_ptr),
    encoder2_ptr(encoder2_ptr),
    motorshield_ptr(motorshield_ptr)
{
    this->leg_conf = leg_conf;  
    this->duty_max = duty_max;
}

void CurrentLoopController::callback()
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    // current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    // velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity   

    currents.current1 = readCurrent(leg_conf.motor1);           // measure current
    joint_states.dth1 = encoder1_ptr->getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
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
        writeMotor(leg_conf.motor1, absDuty1, 1);
    } else { // forwards
        writeMotor(leg_conf.motor1, absDuty1, 0);
    }             
    

    currents.current2 = readCurrent(leg_conf.motor2);      // measure current
    joint_states.dth2 = encoder2_ptr->getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = desired_currents.current2 - currents.current2;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max), -current_int_max);      // anti-windup   
    float ff2 = R*desired_currents.current2 + k_t*joint_states.dth2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp*err_c2 + current_Ki*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycle2);
    if (absDuty2 > duty_max) {
        duty_cycle2 *= duty_max / absDuty2;
        absDuty2 = duty_max;
    }    
    if (duty_cycle2 < 0) { // backwards
        writeMotor(leg_conf.motor2, absDuty2, 1);
    } else { // forwards
        writeMotor(leg_conf.motor2, absDuty2, 0);
    }   

    previous_currents = currents;           
    
}

float CurrentLoopController::readCurrent(motor m) {
    switch (m) {
        case MOTOR_A:
            return -(((float(motorshield_ptr->readCurrentA()) / 65536.0f) * 30.0f) - 15.0f);
        case MOTOR_B:
            return -(((float(motorshield_ptr->readCurrentB()) / 65536.0f) * 30.0f) - 15.0f);
        case MOTOR_C:
            return -(((float(motorshield_ptr->readCurrentC()) / 65536.0f) * 30.0f) - 15.0f);
        case MOTOR_D:
            return -(((float(motorshield_ptr->readCurrentD()) / 65536.0f) * 30.0f) - 15.0f);
        default:
            // Handle unknown case or throw an exception
            return 0.0f;
    }
}

void CurrentLoopController::writeMotor(motor m, float dutyCycle, int direction) {
    switch (m) {
        case MOTOR_A:
            motorshield_ptr->motorAWrite(dutyCycle, direction);
            break;
        case MOTOR_B:
            motorshield_ptr->motorBWrite(dutyCycle, direction);
            break;
        case MOTOR_C:
            motorshield_ptr->motorCWrite(dutyCycle, direction);
            break;
        case MOTOR_D:
            motorshield_ptr->motorDWrite(dutyCycle, direction);
            break;
        default:
            // Handle unknown case or throw an exception
            break;
    }
}
