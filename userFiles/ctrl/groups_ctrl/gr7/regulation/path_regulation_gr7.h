/* Mobile Robots Project - path_regulation_gr7.h
 * regulation to follow a given path
 * Version 1.3
 * Last Update: 18/12/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 

#ifndef _PATH_REGULATION_GR7_H_
#define _PATH_REGULATION_GR7_H_

#include "CtrlStruct_gr7.h"

NAMESPACE_INIT(ctrlGr7);

void follow_path(CtrlStruct *cvs, double theta_path, double linspeed_path, double theta_rob);

NAMESPACE_CLOSE();

#endif
