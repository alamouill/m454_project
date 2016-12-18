
/* Mobile Robots Project - calibration_gr7.h
 * Calibrate the position of the Robot
 * Version 1.2
 * Last Update: 18/12/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 

#ifndef _CALIBRATION_GR7_H_
#define _CALIBRATION_GR7_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr7.h"

NAMESPACE_INIT(ctrlGr7);

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time for flag

	int flag; ///< flag for calibration

} RobotCalibration;

void calibration(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
