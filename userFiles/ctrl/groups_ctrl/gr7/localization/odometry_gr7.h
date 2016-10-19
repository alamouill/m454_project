/*! 
 * \author Group 7
 * \file odometry_gr7.h
 * \brief odometry of the robot
 */

#ifndef _ODOMETRY_GR7_H_
#define _ODOMETRY_GR7_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr7.h"

NAMESPACE_INIT(ctrlGr7);

void update_odometry(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
