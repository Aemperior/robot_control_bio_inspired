#include "matlab.h"

void output_struct2array(struct to_matlab output_struct, float output_data[NUM_OUTPUTS]){
    

    output_data[t_OUT_IDX] = output_struct.t;

    output_data[angleR1_OUT_IDX] = output_struct.jointsR.th1;
    output_data[velocityR1_OUT_IDX] = output_struct.jointsR.dth1;
    output_data[currentR1_OUT_IDX] = output_struct.current_pairR.current1;
    output_data[current_desR1_OUT_IDX] = output_struct.des_current_pairR.current1;
    output_data[duty_cycleR1_OUT_IDX] = output_struct.dutycycleR1;
    
    output_data[angleR2_OUT_IDX] = output_struct.jointsR.th2;
    output_data[velocityR2_OUT_IDX] = output_struct.jointsR.dth2;
    output_data[currentR2_OUT_IDX] = output_struct.current_pairR.current2;
    output_data[current_desR2_OUT_IDX] = output_struct.des_current_pairR.current2;
    output_data[duty_cycleR2_OUT_IDX] = output_struct.dutycycleR2;
    
    output_data[xFootR_OUT_IDX] = output_struct.footR.xFoot;
    output_data[yFootR_OUT_IDX] = output_struct.footR.yFoot;    
    output_data[dxFootR_OUT_IDX] = output_struct.footR.dxFoot;  
    output_data[dyFootR_OUT_IDX] = output_struct.footR.dyFoot;  

    output_data[rDesFootR_x_OUT_IDX] = output_struct.des_footR.xFoot;
    output_data[rDesFootR_y_OUT_IDX] = output_struct.des_footR.yFoot;   
    output_data[vDesFootR_x_OUT_IDX] = output_struct.des_footR.dxFoot;  
    output_data[vDesFootR_y_OUT_IDX] = output_struct.des_footR.dyFoot;


    output_data[angleL1_OUT_IDX] = output_struct.jointsL.th1;
    output_data[velocityL1_OUT_IDX] = output_struct.jointsL.dth1;
    output_data[currentL1_OUT_IDX] = output_struct.current_pairL.current1;
    output_data[current_desL1_OUT_IDX] = output_struct.des_current_pairL.current1;
    output_data[duty_cycleL1_OUT_IDX] = output_struct.dutycycleL1;
    
    output_data[angleL2_OUT_IDX] = output_struct.jointsL.th2;
    output_data[velocityL2_OUT_IDX] = output_struct.jointsL.dth2;
    output_data[currentL2_OUT_IDX] = output_struct.current_pairL.current2;
    output_data[current_desL2_OUT_IDX] = output_struct.des_current_pairL.current2;
    output_data[duty_cycleL2_OUT_IDX] = output_struct.dutycycleL2;
    
    output_data[xFootL_OUT_IDX] = output_struct.footL.xFoot;
    output_data[yFootL_OUT_IDX] = output_struct.footL.yFoot;    
    output_data[dxFootL_OUT_IDX] = output_struct.footL.dxFoot;  
    output_data[dyFootL_OUT_IDX] = output_struct.footL.dyFoot;  

    output_data[rDesFootL_x_OUT_IDX] = output_struct.des_footL.xFoot;
    output_data[rDesFootL_y_OUT_IDX] = output_struct.des_footL.yFoot;   
    output_data[vDesFootL_x_OUT_IDX] = output_struct.des_footL.dxFoot;  
    output_data[vDesFootL_y_OUT_IDX] = output_struct.des_footL.dyFoot;
}

void input_array2struct(float input_params[NUM_INPUTS], struct from_matlab *input_struct_ptr){

    input_struct_ptr->start_period = input_params[start_period_IN_IDX]; 
    input_struct_ptr->start_periodR = input_params[start_periodR_IN_IDX];
    input_struct_ptr->start_periodL = input_params[start_periodL_IN_IDX];
    input_struct_ptr->traj_period = input_params[traj_period_IN_IDX];
    input_struct_ptr->end_period = input_params[end_period_IN_IDX];
    input_struct_ptr-> angleR1_init = input_params[angleR1_init_IN_IDX]; 
    input_struct_ptr-> angleR2_init = input_params[angleR2_init_IN_IDX]; 
    input_struct_ptr-> angleL1_init = input_params[angleL1_init_IN_IDX];
    input_struct_ptr-> angleL2_init = input_params[angleL2_init_IN_IDX];
    input_struct_ptr-> gains = {
        .K_xx = input_params[K_xx_IN_IDX],
        .K_yy = input_params[K_yy_IN_IDX],
        .K_xy = input_params[K_xy_IN_IDX],
        .D_xx = input_params[D_xx_IN_IDX],
        .D_xy = input_params[D_xy_IN_IDX],
        .D_yy = input_params[D_yy_IN_IDX],
    };
    input_struct_ptr-> duty_max = input_params[duty_max_IN_IDX]; 
    
    input_struct_ptr->p_trajR = {
        .x0 = input_params[p_trajR_x0_IN_IDX],
        .y0 = input_params[p_trajR_y0_IN_IDX],
        .rx = input_params[p_trajR_rx_IN_IDX],
        .ry = input_params[p_trajR_ry_IN_IDX],
        .omega = input_params[p_trajR_omega_IN_IDX],
        .phase_delay = input_params[p_trajR_phase_delay_IN_IDX],
    };

    input_struct_ptr->p_trajL = {
        .x0 = input_params[p_trajL_x0_IN_IDX],
        .y0 = input_params[p_trajL_y0_IN_IDX],
        .rx = input_params[p_trajL_rx_IN_IDX],
        .ry = input_params[p_trajL_ry_IN_IDX],
        .omega = input_params[p_trajL_omega_IN_IDX],
        .phase_delay = input_params[p_trajL_phase_delay_IN_IDX],
    };

    input_struct_ptr->traj_mode = input_params[traj_mode_IN_IDX];

    for(int i = 0; i<N_FOOT_POINTS_PER_FOOT;i++) {
      input_struct_ptr->foot_pointsR[i] = input_params[bezier_pointsR_begin_IN_IDX+i];    
    }

    for(int i = 0; i<N_FOOT_POINTS_PER_FOOT;i++) {
      input_struct_ptr->foot_pointsL[i] = input_params[bezier_pointsL_begin_IN_IDX+i];    
    }

}