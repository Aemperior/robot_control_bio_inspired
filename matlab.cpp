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
}

void input_array2struct(float input_params[NUM_INPUTS], struct from_matlab *input_struct_ptr){

    input_struct_ptr->start_period = input_params[start_period_IN_IDX]; 
    input_struct_ptr->traj_period = input_params[traj_period_IN_IDX];
    input_struct_ptr->end_period = input_params[end_period_IN_IDX];
    input_struct_ptr-> angleR1_init = input_params[angleR1_init_IN_IDX]; 
    input_struct_ptr-> angleR2_init = input_params[angleR2_init_IN_IDX]; 
    input_struct_ptr-> gains = {
        .K_xx = input_params[K_xx_IN_IDX],
        .K_yy = input_params[K_yy_IN_IDX],
        .K_xy = input_params[K_xy_IN_IDX],
        .D_xx = input_params[D_xx_IN_IDX],
        .D_xy = input_params[D_xy_IN_IDX],
        .D_yy = input_params[D_yy_IN_IDX],
    };
    input_struct_ptr-> duty_max = input_params[duty_max_IN_IDX]; 
    
    for(int i = 0; i<N_FOOT_POINTS;i++) {
      input_struct_ptr->foot_points[i] = input_params[bezier_points_begin_IN_IDX+i];    
    }

}