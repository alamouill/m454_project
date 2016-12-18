/*! 
 * \author Group 6
 * \file calibration_gr6.h
 * \brief calibration of the robot
 */

#ifndef _CALIBRATION_GR6_H_
#define _CALIBRATION_GR6_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr6.h"

NAMESPACE_INIT(ctrlGr6);

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time for flag

	int flag; ///< flag for calibration

	// calibration position (used because 1 calib for each team)
	double pos_x;
	double pos_y;
	double theta1;
	double theta2;
	double final_theta;
	double rot_speed_r;
	double rot_speed_l;

} RobotCalibration;

void calibration(CtrlStruct *cvs);
void calib_init(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
