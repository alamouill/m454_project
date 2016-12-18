#include "odometry_gr6.h"
#include "useful_gr6.h"
#include "init_pos_gr6.h"
#include <math.h>
#include "kalman_gr6.h"

NAMESPACE_INIT(ctrlGr6);
#define R (30*0.001) // 30 mm
#define B (225*0.001) // 225 mm
#define DEG_TO_RAD (M_PI/180.0)
#define RAD_2_DEG (1/DEG_TO_RAD)

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	double r_sp, l_sp;
	double dt;
	double d_Sr, d_Sl, d_S, d_theta, d_x, d_y;

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
	d_Sr = R*r_sp*dt;	// rad
	d_Sl = R*l_sp*dt;	// rad
	d_S = (d_Sr + d_Sl) / 2; 
	d_theta = ((d_Sr - d_Sl) / B)*RAD_2_DEG; // deg

	d_x = d_S*cos((rob_pos->theta + d_theta/2)*DEG_TO_RAD);
	d_y = d_S*sin((rob_pos->theta + d_theta/2)*DEG_TO_RAD);

	cvs->kalman->ds = d_S;

	rob_pos->x = rob_pos->x + d_x;
	rob_pos->y = rob_pos->y + d_y;
	rob_pos->theta = rob_pos->theta + d_theta; // deg

	// replace the angle between -180 and 180 deg 
	if (rob_pos->theta >= 180)
		rob_pos->theta = rob_pos->theta - 360;
	if (rob_pos->theta < -180)
		rob_pos->theta = rob_pos->theta + 360;

	//printf("odometrie: d_theta = %f\t rob_pos->theta = %f\n", d_theta, rob_pos->theta);
	// ----- odometry computation end ----- //

	// last update time
	rob_pos->last_t = inputs->t;

	//set_plot(rob_pos->x, "x_odo [m]");
	//set_plot(rob_pos->y, "y_odo [m]");
	//set_plot(rob_pos->theta * DEG_TO_RAD, "theta_odo [rad]");
}

NAMESPACE_CLOSE();
