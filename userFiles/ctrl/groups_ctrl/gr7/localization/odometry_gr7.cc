#include "odometry_gr7.h"
#include "useful_gr7.h"
#include "init_pos_gr7.h"
#include <iostream>
#include <math.h>
#define B 0.225
#define R  0.030

NAMESPACE_INIT(ctrlGr7);

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	double r_sp, l_sp;
	double dt;
	double dSr, dSl;
	double dS, dTheta;
	double dX, dY;
	double theta;

	RobotPosition *rob_pos;
	CtrlIn *inputs;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

	r_sp = inputs->r_wheel_speed; // right wheel speed
	l_sp = inputs->l_wheel_speed; // left wheel speed

	// time
	dt = inputs->t - rob_pos->last_t; // time increment since last call

	// safety
	if (dt <= 0.0)
	{
		return;
	}

	// ----- odometry computation start ----- //
	dSr = r_sp*R*dt;
	dSl = l_sp*R*dt;
	dS = (dSr + dSl) / 2;
	dTheta = (dSr - dSl) / B;			//en radian
	theta = rob_pos->theta; //theta en radian
	dX = dS*cos(theta + dTheta / 2);
	dY = dS*sin(theta + dTheta / 2);
	rob_pos->x += dX;
	rob_pos->y += dY;
	rob_pos->theta += dTheta; //rob_pos en rad
	// ----- odometry computation end ----- //
	//set_plot(rob_pos->x, "calculated X");
	//set_plot(rob_pos->y, "calculated Y");
	//set_plot(rob_pos->theta, "calculated theta"); //en rad

	// last update time
	rob_pos->last_t = inputs->t;
}

NAMESPACE_CLOSE();
