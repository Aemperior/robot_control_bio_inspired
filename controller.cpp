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