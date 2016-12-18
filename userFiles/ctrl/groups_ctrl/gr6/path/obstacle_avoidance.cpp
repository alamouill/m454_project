//
//  obstacle_avoidance.cpp
//  
//
//  Created by Lucie Houel on 16/12/2016.
//
//

#include "obstacle_avoidance.hpp"
#include "path_planning_gr6.h"
#include "init_pos_gr6.h"
#include "opp_pos_gr6.h"
#include "useful_gr6.h"
#include "speed_regulation_gr6.h"
#include <math.h>
#include "strategy_gr6.h"
#include "path_regulation_gr6.h"

NAMESPACE_INIT(ctrlGr6);
#define DIST_MIN 0.157 // distance minimal to the obstacles (m)
#define R_OPP 0.182
#define L_SAFE (3*DIST_MIN)
#define K_REP 0.1
#define K_ATT 0.8
#define K_W 0.2


// Function to calculate only one of the projected component of the repulsive force dependant on the rob_one_coord and obs_one_coord (value of x or y component)
float repulsive_force(float distance, float distance_min, float rob_one_coord, float obs_one_coord){
    // Initialization
    float f_rep;
    
    // zero repulsive force if the obstacle is too far
    if (distance < distance_min){
        f_rep=0;
    }
    // hyperbolic ?? force otherwise
    else {
        f_rep=K_REP*(1/distance-1/distance_min)*(rob_one_coord-obs_one_coord)/pow(distance,3);
    }
    return f_rep;
}

float attractive_force(float rob_one_coord, float target_one_coord){
    // Initialization
    float f_att;
    
    // Calculation
    f_att=-K_ATT*(rob_one_coord-target_one_coord);
    
    return f_att;
}

void total_force(CtrlStruct *cvs){
  
    // Declaration
    RobotPosition *rob_pos;
    OpponentsPosition *opp_pos;
    PathPlanning *path;
    Obstacle *obs;
    int i=0,j=0;
    int nb_opp;
    float f_rep_x;
    float f_rep_y;
    float f_att_x;
    float f_att_y;
    double distance; //???????? double ?!?
    
    // Initialization
    rob_pos=cvs->rob_pos;
    opp_pos=cvs->opp_pos;
    path=cvs->path;
    nb_opp=opp_pos->nb_opp;
    f_rep_x=0;
    f_rep_y=0;
    f_att_x=0;
    f_att_y=0;
    
    // Repulsive force calculation
    for (i=0; i<(NB_OBSTACLE); i++) {
        
        obs=path->obstacle_list[i];
        distance_to_obstacle(rob_pos->x, rob_pos->y, obs);
        f_rep_x=f_rep_x+repulsive_force(obs->distance, L_SAFE, rob_pos->x, obs->proxima_x);
        f_rep_y=f_rep_y+repulsive_force(obs->distance, L_SAFE, rob_pos->y, obs->proxima_y);
    }
    
    if (nb_opp > 0) {
        for (j=0; j<nb_opp; j++) {
            distance=norm_dist( rob_pos->x-opp_pos->x[i], rob_pos->y-opp_pos->y[i]);
            f_rep_x=f_rep_x+repulsive_force(distance, R_OPP+L_SAFE, rob_pos->x, opp_pos->x[i]);
            f_rep_y=f_rep_y+repulsive_force(distance, R_OPP+L_SAFE, rob_pos->y, opp_pos->y[i]);
        }
    }
    
    // Attractive force calculation
    f_att_x=attractive_force(rob_pos->x, path->nextNode->posx);
    f_att_y=attractive_force(rob_pos->y, path->nextNode->posy);
    
    // Update of the force in the path_structure
    path->f_rep_x=f_rep_x;
    path->f_rep_y=f_rep_y;
    path->f_att_x=f_att_x;
    path->f_att_y=f_att_y;
    
    return;
}

void command_potential(CtrlStruct *cvs) {
    // Declaration
    RobotPosition *rob_pos;
    PathPlanning *path;
    //float K=0.1; // proportionnal factor between the force and the speed
    float command_v;
    float command_w;
    //double dt;
    
    // Initialization
    path=cvs->path;
    rob_pos=cvs->rob_pos;
    
    // Calculation
    total_force(cvs);
    
    // time
    //dt = cvs->inputs->t - path->last_t; // time interval since last call
    
    // test de pseudo-integration
    //command_v = (path->command_v) + norm_dist((path->f_rep_x + path->f_att_x),(path->f_rep_y + path->f_att_y))*dt;
    //command_w = (path->command_w) + atan2((path->f_rep_y + path->f_att_y), (path->f_rep_x + path->f_att_x))*dt;
    
    command_v= norm_dist((path->f_rep_x + path->f_att_x),(path->f_rep_y + path->f_att_y));
	command_w= K_W*atan2((path->f_rep_y + path->f_att_y), (path->f_rep_x + path->f_att_x));
    
    // Update path structure
    path->command_v=command_v;
    path->command_w=command_w;
    
    // last update time
    //path->last_t = cvs->inputs->t;
    
    return;
    
}

bool check_obstacle_front(CtrlStruct *cvs){
    // Declaration
    RobotPosition *rob_pos;
    PathPlanning *path;
    Obstacle *obs;
    bool obs_in_front;
    int i=0;
    
    // Initialisation
    rob_pos= cvs->rob_pos;
    path=cvs->path;
    obs_in_front=false;
    
    // Calculation
    for (i=0; i<(NB_OBSTACLE); i++) {
        
        obs=path->obstacle_list[i];
        distance_to_obstacle(rob_pos->x, rob_pos->y, obs);
        if (obs->distance < DIST_MIN){
            obs_in_front=true;
        }
    }
    
    return obs_in_front;
}

NAMESPACE_CLOSE();
