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
#include "matlab.h"
#include "controller.h"

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); //initialize the motor shield with a period of 12000 ticks or ~20kHZ
Ticker currentLoop;

Matrix Jacobian(2,2);
Matrix JacobianT(2,2);
Matrix temp_product(2,2);

// Variables for q1
float current1;
float current_des1 = 0;
float prev_current_des1 = 0;
float current_int1 = 0;
float angle1;
float velocity1;
float duty_cycleR1;
float angle1_init;

// Variables for q2
float current2;
float current_des2 = 0;
float prev_current_des2 = 0;
float current_int2 = 0;
float angle2;
float velocity2;
float duty_cycleR2;
float angle2_init;

#define N_param 18.75
const float Ir_param = 0.0035/pow(N_param,2);
const kinematic_params params = {
    .l_OA=.011, 
    .l_OB=.042, 
    .l_AC=.096, 
    .l_DE=.091,
    .m1 =.0393 + .2,
    .m2 =.0368, 
    .m3 = .00783,
    .m4 = .0155,
    .I1 = 0.0000251,  //25.1 * 10^-6,
    .I2 = 0.0000535,  //53.5 * 10^-6,
    .I3 = 0.00000925, //9.25 * 10^-6,
    .I4 = 0.0000222,  //22.176 * 10^-6,
    .l_O_m1=0.032,
    .l_B_m2=0.0344, 
    .l_A_m3=0.0622,
    .l_C_m4=0.0610,
    .N = N_param,
    .Ir = Ir_param,
};

// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float impedance_control_period_us = 1000.0f;  // 1kHz impedance control loop
float start_period, traj_period, end_period;

// Control parameters
float current_Kp = 4.0f;         
float current_Ki = 0.4f;           
float current_int_max = 3.0f;       
float duty_max;      

struct leg_gain gains;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction

// Current control interrupt function
void CurrentLoop()
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    // current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    // velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity   
    current1 = -(((float(motorShield.readCurrentD())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity1 = encoderD.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1 = current_des1 - current1;                                             // current errror
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
    current2     = -(((float(motorShield.readCurrentC())/65536.0f)*30.0f)-15.0f);       // measure current
    velocity2 = encoderC.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = current_des2 - current2;                                             // current error
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

int main (void)
{
    
    // Object for 7th order Cartesian foot trajectory
    BezierCurve rDesFoot_bez(2,BEZIER_ORDER_FOOT);
    
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    pc.printf("%f",input_params[0]);
    
    while(1) {
        
        // If there are new inputs, this code will run
        if (server.getParams(input_params,NUM_INPUTS)) {
            
            struct from_matlab input_struct; 
            input_array2struct(input_params, &input_struct); 

            start_period    = input_struct.start_period;
            traj_period     = input_struct.traj_period;    // Trajectory time/length
            end_period      = input_struct.end_period;    // Second buffer time, after trajectory
    
            angle1_init     = input_struct.angleR1_init;    // Initial angle for q1 (rad)
            angle2_init     = input_struct.angleR2_init;    // Initial angle for q2 (rad)

            gains           = input_struct.gains; 
            
            duty_max        = input_struct.duty_max;   // Maximum duty factor

            rDesFoot_bez.setPoints(input_struct.foot_points);
            
            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop,current_control_period_us);
                        
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            encoderC.reset();
            encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorDWrite(0, 0);
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0);
                         
            // Run experiment
            while( t.read() < start_period + traj_period + end_period) { 
                 
                // Read encoders to get motor states
                // angle1 = encoderA.getPulses() *PULSE_TO_RAD + angle1_init;   
                // velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;    
                angle1 = encoderD.getPulses() *PULSE_TO_RAD + angle1_init;    
                velocity1 = encoderD.getVelocity() * PULSE_TO_RAD;
                 
                // angle2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;       
                // velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;         
                angle2 = encoderC.getPulses() * PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderC.getVelocity() * PULSE_TO_RAD;   
                
                const float th1 = angle1;
                const float th2 = angle2;
                const float dth1= velocity1;
                const float dth2= velocity2;

                joint_state joints_R_state = {
                    .th1 = th1,
                    .th2 = th2,
                    .dth1 = dth1,
                    .dth2 = dth2,
                };
 
                // Calculate the Jacobian  
                foot_jacobian J = calc_foot_jacobi(th1, th2, params); 
                                
                // Calculate the forward kinematics (position and velocity)
                foot_state foot_R_state= calc_forward_kinematics(joints_R_state, params); 

                // Set gains based on buffer and traj times, then calculate desired x,y from Bezier trajectory at current time if necessary
                float teff  = 0;
                float vMult = 0;
                if( t < start_period) {
                    // Why does original code do something with gains here?
                    teff = 0;
                }
                else if (t < start_period + traj_period)
                {
                    gains = input_struct.gains; 
                    teff = (t-start_period);
                    vMult = 1;
                }
                else
                {
                    teff = traj_period;
                    vMult = 0;
                }
                
                // Get desired foot positions and velocities
                float rDesFoot[2] , vDesFoot[2];
                rDesFoot_bez.evaluate(teff/traj_period,rDesFoot);
                rDesFoot_bez.evaluateDerivative(teff/traj_period,vDesFoot);
                vDesFoot[0]/=traj_period;
                vDesFoot[1]/=traj_period;
                vDesFoot[0]*=vMult;
                vDesFoot[1]*=vMult;
                
                // Calculate the inverse kinematics (joint positions and velocities) for desired joint angles 
                foot_state desired_foot_state = {
                    .xFoot = rDesFoot[0],
                    .yFoot = rDesFoot[1],
                    .dxFoot = vDesFoot[0],
                    .dyFoot = vDesFoot[1],
                };

                joint_state desired_joint_state = calc_desired_joints(desired_foot_state, J, params); 
                float th1_des = desired_joint_state.th1;
                float th2_des = desired_joint_state.th2;
                float dth1_des = desired_joint_state.dth1;
                float dth2_des = desired_joint_state.dth2;

                current_pair desired_current = get_desired_current(joints_R_state, gains, desired_joint_state, k_t);
                current_des1 = desired_current.current1;
                current_des2 = desired_current.current2;

                // Form output to send to MATLAB     
                float output_data[NUM_OUTPUTS];
                
                struct to_matlab output_struct; 

                output_struct.t = t.read();
                output_struct.footR = foot_R_state;
                output_struct.jointsR = joints_R_state;
                output_struct.des_footR = desired_foot_state;
                output_struct.des_jointsR = desired_joint_state;    
                output_struct.current_pairR = {
                    .current1 = current1,
                    .current2 = current2,
                };
                output_struct.des_current_pairR = {
                    .current1 = current_des1,
                    .current2 = current_des2,
                };
                output_struct.dutycycleR1 = duty_cycleR1;
                output_struct.dutycycleR2 = duty_cycleR2;

                output_struct2array(output_struct, output_data);
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);

                wait_us(impedance_control_period_us);   
            }
            
            // Cleanup after experiment
            server.setExperimentComplete();
            currentLoop.detach();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorDWrite(0, 0);
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0);
        
        } // end if
        
    } // end while
    
} // end main

