/*! 
 * \author Group 7
 * \file calibration_gr7.h
 * \brief calibration of the robot
 */

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
