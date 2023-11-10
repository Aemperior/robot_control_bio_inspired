#include "controller.h"

// struct current_pair get_desired_current(struct joint_state state, struct joint_state desired_state){

//     float current_des1 = (K_xx*(th1_des - th1) + D_xx*(dth1_des - dth1))/k_t; 
//     float current_des2 = (K_yy*(th2_des - th2) + D_yy*(dth2_des - dth2))/k_t;  

//     struct current_pair desired_current_pair = {
//         .current1 = current_des1,
//         .current2 = current_des2,
//     };

//     return desired_current_pair; 
// }