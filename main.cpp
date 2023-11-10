#include "pragma_replacement.h"

#include "kinematics.h"
#include "matlab.h"
#include "controller.h"

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

Ticker currentLoopR;
Ticker currentLoopL;

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
float start_period, start_periodR, start_periodL, traj_period, end_period;     

struct leg_gain gains;

float angleR1_init;
float angleR2_init; 
float angleL1_init;
float angleL2_init; 
float duty_max; 

int main (void)
{
    QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
    QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
    QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
    QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

    MotorShield motorShield(24000); //initialize the motor shield with a period of 12000 ticks or ~20kHZ
    
    // Object for 7th order Cartesian foot trajectory
    BezierCurve rDesFootR_bez(2,BEZIER_ORDER_FOOT);
    BezierCurve rDesFootL_bez(2,BEZIER_ORDER_FOOT);
    
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
            start_periodR   = input_struct.start_periodR;
            start_periodL   = input_struct.start_periodL;
            traj_period     = input_struct.traj_period;    // Trajectory time/length
            end_period      = input_struct.end_period;    // Second buffer time, after trajectory
    
            angleR1_init    = input_struct.angleR1_init;    // Initial angle for q1 (rad)
            angleR2_init    = input_struct.angleR2_init;    // Initial angle for q2 (rad)

            angleL1_init    = input_struct.angleL1_init;    // Initial angle for q1 (rad)
            angleL2_init    = input_struct.angleL2_init;    // Initial angle for q2 (rad)

            gains           = input_struct.gains; 
            
            duty_max        = input_struct.duty_max;   // Maximum duty factor

            rDesFootR_bez.setPoints(input_struct.foot_pointsR);
            rDesFootL_bez.setPoints(input_struct.foot_pointsL);
            
            CurrentLoopController current_controllerR(
                duty_max,
                {
                    .motor1 = MOTOR_D,
                    .motor2 = MOTOR_C,},
                &encoderD,
                &encoderC,
                &motorShield);

            CurrentLoopController current_controllerL(
                duty_max,
                {
                    .motor1 = MOTOR_A,
                    .motor2 = MOTOR_B,},
                &encoderA,
                &encoderB,
                &motorShield);

            currentLoopR.attach_us(callback(&current_controllerR, &CurrentLoopController::callback),current_control_period_us);
            currentLoopL.attach_us(callback(&current_controllerL, &CurrentLoopController::callback),current_control_period_us);

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
                joint_state joints_R_state = {
                    .th1 = encoderD.getPulses() *PULSE_TO_RAD + angleR1_init,
                    .th2 = encoderC.getPulses() * PULSE_TO_RAD + angleR2_init,
                    .dth1 = encoderD.getVelocity() * PULSE_TO_RAD,
                    .dth2 = encoderC.getVelocity() * PULSE_TO_RAD,
                };

                joint_state joints_L_state = {
                    .th1 = encoderA.getPulses() *PULSE_TO_RAD + angleL1_init,
                    .th2 = encoderB.getPulses() * PULSE_TO_RAD + angleL2_init,
                    .dth1 = encoderA.getVelocity() * PULSE_TO_RAD,
                    .dth2 = encoderB.getVelocity() * PULSE_TO_RAD,
                };
 
                // Calculate the Jacobian  
                foot_jacobian J_R = calc_foot_jacobi(joints_R_state.th1, joints_R_state.th2, params); 
                foot_jacobian J_L = calc_foot_jacobi(joints_L_state.th1, joints_L_state.th2, params); 
                                
                // Calculate the forward kinematics (position and velocity)
                foot_state foot_R_state= calc_forward_kinematics(joints_R_state, params);                
                foot_state foot_L_state= calc_forward_kinematics(joints_L_state, params); 

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
                float rDesFootR[2] , vDesFootR[2];
                rDesFootR_bez.evaluate(teff/traj_period,rDesFootR);
                rDesFootR_bez.evaluateDerivative(teff/traj_period,vDesFootR);
                vDesFootR[0]/=traj_period;
                vDesFootR[1]/=traj_period;
                vDesFootR[0]*=vMult;
                vDesFootR[1]*=vMult;

                float rDesFootL[2] , vDesFootL[2];
                rDesFootL_bez.evaluate(teff/traj_period,rDesFootL);
                rDesFootL_bez.evaluateDerivative(teff/traj_period,vDesFootL);
                vDesFootL[0]/=traj_period;
                vDesFootL[1]/=traj_period;
                vDesFootL[0]*=vMult;
                vDesFootL[1]*=vMult;
                
                // Calculate the inverse kinematics (joint positions and velocities) for desired joint angles 

                struct foot_state desired_foot_stateR = {
                    .xFoot = rDesFootR[0],
                    .yFoot = rDesFootR[1],
                    .dxFoot = vDesFootR[0],
                    .dyFoot = vDesFootR[1],
                };
                // struct foot_state desired_foot_stateR = foot_R_state; 
                struct foot_state desired_foot_stateL = {
                    .xFoot = rDesFootL[0],
                    .yFoot = rDesFootL[1],
                    .dxFoot = vDesFootL[0],
                    .dyFoot = vDesFootL[1],
                };
                // struct foot_state desired_foot_stateL = foot_R_state; 

                struct joint_state desired_joint_stateR = calc_desired_joints(desired_foot_stateR, J_R, params); 
                struct joint_state desired_joint_stateL = calc_desired_joints(desired_foot_stateL, J_L, params); 

                struct current_pair desired_currentR = get_desired_current(joints_R_state, gains, desired_joint_stateR, current_controllerR.k_t);
                current_controllerR.desired_currents = desired_currentR;

                struct current_pair desired_currentL = get_desired_current(joints_L_state, gains, desired_joint_stateL, current_controllerL.k_t);
                current_controllerL.desired_currents = desired_currentL;

                // Form output to send to MATLAB     
                float output_data[NUM_OUTPUTS];
                
                struct to_matlab output_struct; 

                output_struct.t = t.read();

                output_struct.footR = foot_R_state;
                output_struct.jointsR = joints_R_state;
                output_struct.des_footR = desired_foot_stateR;
                output_struct.des_jointsR = desired_joint_stateR;    
                output_struct.current_pairR = current_controllerR.currents;
                output_struct.des_current_pairR = current_controllerR.desired_currents;
                output_struct.dutycycleR1 = current_controllerR.duty_cycle1;
                output_struct.dutycycleR2 = current_controllerR.duty_cycle2;

                output_struct.footL = foot_L_state;
                output_struct.jointsL = joints_L_state;
                output_struct.des_footL = desired_foot_stateL;
                output_struct.des_jointsL = desired_joint_stateL;    
                output_struct.current_pairL = current_controllerL.currents;
                output_struct.des_current_pairL = current_controllerL.desired_currents;
                output_struct.dutycycleL1 = current_controllerL.duty_cycle1;
                output_struct.dutycycleL2 = current_controllerL.duty_cycle2;

                output_struct2array(output_struct, output_data);
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);

                wait_us(impedance_control_period_us);   
            }
            
            // Cleanup after experiment
            server.setExperimentComplete();
            currentLoopR.detach();
            currentLoopL.detach();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorDWrite(0, 0);
            motorShield.motorBWrite(0, 0); //turn motor B off
            motorShield.motorCWrite(0, 0);
        
        } // end if
        
    } // end while
    
} // end main

