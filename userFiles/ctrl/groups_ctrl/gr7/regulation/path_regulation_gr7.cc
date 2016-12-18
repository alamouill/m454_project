/* Mobile Robots Project - path_regulation_gr7.cc
 * regulation to follow a given path
 * Version 2.1
 * Last Update: 18/12/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 


#include "path_regulation_gr7.h"
#include "useful_gr7.h"
#include "speed_regulation_gr7.h"

NAMESPACE_INIT(ctrlGr7);

#define KP_NORM 0.15		//0.2
#define KI_NORM 0.01	//0.01
#define KP_THETA 30		//20
#define KI_THETA 0.1	// 0.2

// This function determine the way the robot will move (turn + backward / forward)
static void determine_dynamic(double* linspeed_path, double* err_theta)
{
	// To turn both ways (Right or left)
	if (fabs(*err_theta) > M_PI)
		*err_theta -= sgn(*err_theta) * 2 * M_PI;
	// Does it have to go backward ?
	if (fabs(*err_theta) > M_PI / 2)
	{
		*linspeed_path = -*linspeed_path;
		*err_theta -= sgn(*err_theta) * M_PI;
	}
	return;
}

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 * 
 * This function uses a PI controller on the linear speed and the angle of the robot to compute the new speed of the wheels.
 */
void follow_path(CtrlStruct *cvs, double theta_path, double linspeed_path, double theta_rob)
{	
	// Definitions
	CtrlIn *inputs = cvs->inputs;
	RobotPosition *rob_pos = cvs->rob_pos;
	// Variables
	double linspeed_act, linspeed, err_linspeed, err_theta, omega;
	static double int_err_linspeed = 0, int_err_theta = 0;

	// Computes the actual linear speed
	linspeed_act = (inputs->r_wheel_speed + inputs->r_wheel_speed) / 2;

	// Computes the errors for the PI
	err_theta = theta_path - theta_rob;
	// Determine if the robot has to go backward and the direction he has to turned
	determine_dynamic(&linspeed_path, &err_theta);
	err_linspeed = linspeed_path - linspeed_act;

	// Computes the integtral terms of the PI
	int_err_linspeed += err_linspeed;
	int_err_linspeed = limit_range(int_err_linspeed, -40 / KI_NORM, 40 / KI_NORM);
	int_err_theta += err_theta;
	int_err_theta = limit_range(int_err_theta, -3, 3);

	// PI
	linspeed = KP_NORM*err_linspeed + KI_NORM*int_err_linspeed;
	omega = KP_THETA*err_theta + KI_THETA*int_err_theta;

	// Set the wheel speeds
	speed_regulation(cvs, linspeed + omega, linspeed - omega);
	return;
}

NAMESPACE_CLOSE();
