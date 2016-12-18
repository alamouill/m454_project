/*! 
 * \author Group 6
 * \file odometry_gr6.h
 * \brief odometry of the robot
 */

#ifndef _ODOMETRY_GR6_H_
#define _ODOMETRY_GR6_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr6.h"

NAMESPACE_INIT(ctrlGr6);

void update_odometry(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
