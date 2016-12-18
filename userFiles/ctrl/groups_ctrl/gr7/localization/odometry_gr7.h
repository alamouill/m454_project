/* Mobile Robots Project - odometry_gr7.cc
 * Estimate Robot Position Using Odometry
 * Version 1.2
 * Last Update: 8/11/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 

#ifndef _ODOMETRY_GR7_H_
#define _ODOMETRY_GR7_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr7.h"
#include <stdio.h>

NAMESPACE_INIT(ctrlGr7);

void update_odometry(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
