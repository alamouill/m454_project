#include "opp_pos_gr6.h"
#include "init_pos_gr6.h"
#include "useful_gr6.h"
#include <kalman_gr6.h>
#include <math.h>

NAMESPACE_INIT(ctrlGr6);

/*! \brief compute the opponents position using the tower
*
* \param[in,out] cvs controller main structure
*/
void opponents_tower(CtrlStruct *cvs)
{
	// variables declaration
	int nb_opp;
	int rise_index_1, rise_index_2, fall_index_1, fall_index_2;

	double delta_t;
	double rise_1, rise_2, fall_1, fall_2;
	double tau = 0.1;
	double prev_opp_pos_x0, prev_opp_pos_x1, prev_opp_pos_y0, prev_opp_pos_y1;
	double test_pos_opp_x0=0, test_pos_opp_y0=0, test_pos_opp_x1=0, test_pos_opp_y1=0;
	double err_opp_0, err_opp_1;

	CtrlIn *inputs;
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;

	// variables initialization
	inputs = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;

	nb_opp = opp_pos->nb_opp;

	// store previous opponnent position
	prev_opp_pos_x0 = opp_pos->x[0];
	prev_opp_pos_x1 = opp_pos->x[1];
	prev_opp_pos_y0 = opp_pos->y[0];
	prev_opp_pos_y1 = opp_pos->y[1];

	// no opponent
	if (!nb_opp)
	{
		return;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	// low pass filter time increment ('delta_t' is the last argument of the 'first_order_filter' function)
	delta_t = inputs->t - opp_pos->last_t;
	opp_pos->last_t = inputs->t;

	// indexes
	rise_index_1 = inputs->rising_index;
	fall_index_1 = inputs->falling_index;

	// rise and fall angles of the first opponent
	rise_1 = inputs->last_rising[rise_index_1];
	fall_1 = inputs->last_falling[fall_index_1];

	// rise and fall angles of the second opponent
	if (nb_opp == 2)
	{
		rise_index_2 = (rise_index_1 - 1 < 0) ? NB_STORE_EDGE - 1 : rise_index_1 - 1;
		fall_index_2 = (fall_index_1 - 1 < 0) ? NB_STORE_EDGE - 1 : fall_index_1 - 1;

		rise_2 = inputs->last_rising[rise_index_2];
		fall_2 = inputs->last_falling[fall_index_2];
	}

	// ----- opponents position computation start ----- //
	if (nb_opp == 2) {
		// check for the order of opponenent
		if (single_opp_tower(rise_2, fall_2, rob_pos->x, rob_pos->y, (rob_pos->theta)*DEG_TO_RAD, &test_pos_opp_x0, &test_pos_opp_y0)) {
			if (single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, (rob_pos->theta)*DEG_TO_RAD, &test_pos_opp_x1, &test_pos_opp_y1)) {
				// erreur based on distance
				err_opp_0 = sqrt(pow(test_pos_opp_x0 - prev_opp_pos_x1, 2) + pow(test_pos_opp_y0 - prev_opp_pos_y1, 2));
				err_opp_1 = sqrt(pow(test_pos_opp_x1 - prev_opp_pos_x1, 2) + pow(test_pos_opp_y1 - prev_opp_pos_y1, 2));

				// if err0 smaller than err1, then rise2 is for rise1 and vice-versa
				if (err_opp_0 < err_opp_1) {
					opp_pos->x[0] = first_order_filter(prev_opp_pos_x0, test_pos_opp_x1, tau, delta_t);
					opp_pos->y[0] = first_order_filter(prev_opp_pos_y0, test_pos_opp_y1, tau, delta_t);
					opp_pos->x[1] = first_order_filter(prev_opp_pos_x1, test_pos_opp_x0, tau, delta_t);
					opp_pos->y[1] = first_order_filter(prev_opp_pos_y1, test_pos_opp_y0, tau, delta_t);
				}
				else {
					opp_pos->x[0] = first_order_filter(prev_opp_pos_x0, test_pos_opp_x0, tau, delta_t);
					opp_pos->y[0] = first_order_filter(prev_opp_pos_y0, test_pos_opp_y0, tau, delta_t);
					opp_pos->x[1] = first_order_filter(prev_opp_pos_x1, test_pos_opp_x1, tau, delta_t);
					opp_pos->y[1] = first_order_filter(prev_opp_pos_y1, test_pos_opp_y1, tau, delta_t);
				}
			}
		}
		// if computation not correct, then just keep last value
		else {
			opp_pos->x[1] = prev_opp_pos_x1;
			opp_pos->y[1] = prev_opp_pos_y1;
			opp_pos->x[0] = prev_opp_pos_x0;
			opp_pos->y[0] = prev_opp_pos_y0;
		}
		/*
		set_plot(opp_pos->x[0], " opp_pos0 x[m] ");
		set_plot(opp_pos->y[0], " opp_pos0 y[m] ");		set_plot(opp_pos->x[1], " opp_pos1 x[m] ");
		set_plot(opp_pos->y[1], " opp_pos1 y[m] ");
		*/
		
	}
	else {
		if (single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, (rob_pos->theta)*DEG_TO_RAD, &(opp_pos->x[0]), &(opp_pos->y[0]))) {
			opp_pos->x[0] = first_order_filter(prev_opp_pos_x0, opp_pos->x[0], tau, delta_t);
			opp_pos->y[0] = first_order_filter(prev_opp_pos_y0, opp_pos->y[0], tau, delta_t);
		}
		else {
			opp_pos->x[0] = prev_opp_pos_x0;
			opp_pos->y[0] = prev_opp_pos_y0;
		}
		
		set_plot(opp_pos->x[0], " opp_pos0 x[m] ");
		set_plot(opp_pos->y[0], " opp_pos0 y[m] ");
		/**/
	}
	

	// ----- opponents position computation end ----- //
}

/*! \brief compute a single opponent position
*
* \param[in] last_rise last rise relative angle [rad]
* \param[in] last_fall last fall relative angle [rad]
* \param[in] rob_x robot x position [m]
* \param[in] rob_y robot y position [m]
* \param[in] rob_theta robot orientation [rad]
* \param[out] new_x_opp new known x opponent position
* \param[out] new_y_opp new known y opponent position
* \return 1 if computation successful, 0 otherwise
*/
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp)
{
	*new_x_opp = 0.0;
	*new_y_opp = 0.0;
	double alpha, dist_rel, theta_rel;
	if (last_rise > last_fall) {
		last_rise = last_rise - M_PI*2;
	}

	alpha = last_fall - last_rise; // angle in rad of the opponent tower
	dist_rel = fabs(TOWER_RADIUS / sin(alpha / 2)); // distance to the center of the tower from tower to tower
	theta_rel = last_rise + alpha / 2;	// relative angle from the center of the tower to the tower

	// wrong calculation or angle rise and fall to close or not calculabe
	if (dist_rel > 4) {
		return 0;
	}

	*new_x_opp = rob_x + DIST_ROB_TOW*cos(rob_theta) + dist_rel*cos(rob_theta + theta_rel);
	*new_y_opp = rob_y + DIST_ROB_TOW*sin(rob_theta) + dist_rel*sin(rob_theta + theta_rel);

	//printf("opp_pos:\t x_opp_tow=\t%f \t y_opp_tow=\t%f \n", *new_x_opp, *new_y_opp);
	//printf("opp_pos:\t last_fall=%f\t,last_rise=%f,x_rob=%f,y_rob=%f,theta_rob=%f\n", last_fall,last_rise, rob_x, rob_y, rob_theta);

	return 1;
}

/*! \brief check if there is an opponent in front of the robot
*
* \param[in] cvs controller main structure
* \return 1 if opponent robot in front of the current robot
*/
int check_opp_front(CtrlStruct *cvs)
{
	// variables declaration
	int i=0, nb_opp=0;
	double dx=0, dy=0, dist = 0;
	double kal_x, kal_y;
	double speed_factor;

	OpponentsPosition *opp_pos;

	// variables initialization
	opp_pos = cvs->opp_pos;
	nb_opp = opp_pos->nb_opp;
	kal_x = cvs->kalman->kalman_pos.x();
	kal_y = cvs->kalman->kalman_pos.y();

	// calcul of speed factor to take into account the speed when seing the opponnent
	speed_factor = fabs((cvs->kalman->ds) * 3.5 / 0.0005);
	if (speed_factor<1.5) {
		speed_factor = 1.5;
	}
	//printf("speed factor : %f\n", speed_factor);

	// no opponent
	if (!nb_opp)
	{
		return 0;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	for (i = 0; i<nb_opp; i++)
	{
		// ----- opponents check computation start ----- //
		dx = opp_pos->x[i] - kal_x;
		dy = opp_pos->y[i] - kal_y;
		dist = sqrt(dx*dx + dy*dy);
		/*set_plot(speed_factor*R_PROTECTION_CIRCLE, "treshold");
		if(i==0)
			set_plot(dist, "opp1_d");
		else
			set_plot(dist, "opp2_d");
			*/
		if (dist <= speed_factor*R_PROTECTION_CIRCLE) {
			return 1;
		}
			
		// ----- opponents check computation end ----- //
	}

	return 0;
}

NAMESPACE_CLOSE();
