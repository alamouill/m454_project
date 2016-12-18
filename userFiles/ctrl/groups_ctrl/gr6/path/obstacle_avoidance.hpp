//
//  obstacle_avoidance.hpp
//  
//
//  Created by Lucie Houel on 16/12/2016.
//
//

#ifndef obstacle_avoidance_hpp
#define obstacle_avoidance_hpp

#include <stdio.h>
#include "namespace_ctrl.h"
#include "CtrlStruct_gr6.h"

NAMESPACE_INIT(ctrlGr6);

float repulsive_force(float distance, float distance_min, float rob_one_coord, float obs_one_coord);
void total_force(CtrlStruct *cvs);
float attractive_force(float rob_one_coord, float target_one_coord);
void command_potential(CtrlStruct *cvs);
bool check_obstacle_front (CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif /* obstacle_avoidance_hpp */

