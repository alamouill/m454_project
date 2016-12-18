/* Mobile Robots Project - opp_pos_gr7.h
 * Estimate Opponnents Robot Position Using the beacon tower
 * Version 1.7
 * Last Update: 8/11/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 

#ifndef _OPP_POS_GR7_H_
#define _OPP_POS_GR7_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr7.h"

NAMESPACE_INIT(ctrlGr7);

/// robot position
typedef struct OpponentsPosition
{
	double x[2]; ///< x position of opponents [m]
	double y[2]; ///< y position of opponents [m]

	double last_t; ///< last time the filters were updated [s]

	int nb_opp; ///< number of opponents

} OpponentsPosition;

// function prototype
int check_opp_front(CtrlStruct *cvs);
void opponents_tower(CtrlStruct *cvs);
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp);

NAMESPACE_CLOSE();

#endif
