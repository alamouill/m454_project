#include "opp_pos_gr7.h"
#include "init_pos_gr7.h"
#include "useful_gr7.h"
#include <math.h>
#include <iostream>

#define ROB_TOW 0.083

NAMESPACE_INIT(ctrlGr7);

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

	//x: index 0, y: index 1
	static double last_opp_0[2] = { 0,0 };
	static double last_opp_1[2] = { 0,0 };

	CtrlIn *inputs;
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;

	// variables initialization
	inputs = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;



	nb_opp = opp_pos->nb_opp;


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

		single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &(opp_pos->x[0]), &(opp_pos->y[0]));

		single_opp_tower(rise_2, fall_2, rob_pos->x, rob_pos->y, rob_pos->theta, &(opp_pos->x[1]), &(opp_pos->y[1]));

		double posRob0[] = { opp_pos->x[0],opp_pos->y[0] };
		double posRob1[] = { opp_pos->x[1],opp_pos->y[1] };

		// find distances of the robots to the last position saved
		double distVar0 = get_Distance(posRob0, last_opp_0) + get_Distance(posRob1, last_opp_1);
		double distVar1 = get_Distance(posRob0, last_opp_1) + get_Distance(posRob1, last_opp_0);

		// map last positions to new ones with minimal displacement between them
		// do first order filter
		// update last values accordingly
		if (distVar0<distVar1) {
			opp_pos->x[0] = first_order_filter(last_opp_0[0], opp_pos->x[0], 0.2, delta_t);
			opp_pos->y[0] = first_order_filter(last_opp_0[1], opp_pos->y[0], 0.2, delta_t);

			opp_pos->x[1] = first_order_filter(last_opp_1[0], opp_pos->x[1], 0.2, delta_t);
			opp_pos->y[1] = first_order_filter(last_opp_1[1], opp_pos->y[1], 0.2, delta_t);

			last_opp_0[0] = opp_pos->x[0];
			last_opp_0[1] = opp_pos->y[0];
			last_opp_1[0] = opp_pos->x[1];
			last_opp_1[1] = opp_pos->y[1];
		}
		else {
			opp_pos->x[0] = first_order_filter(last_opp_1[0], opp_pos->x[0], 0.2, delta_t);
			opp_pos->y[0] = first_order_filter(last_opp_1[1], opp_pos->y[0], 0.2, delta_t);

			opp_pos->x[1] = first_order_filter(last_opp_0[0], opp_pos->x[1], 0.2, delta_t);
			opp_pos->y[1] = first_order_filter(last_opp_0[1], opp_pos->y[1], 0.2, delta_t);

			last_opp_0[0] = opp_pos->x[1];
			last_opp_0[1] = opp_pos->y[1];
			last_opp_1[0] = opp_pos->x[0];
			last_opp_1[1] = opp_pos->y[0];
		}

	}
	else {
		// case 1 oponent
		single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &(opp_pos->x[0]), &(opp_pos->y[0]));
		opp_pos->x[0] = first_order_filter(last_opp_0[0], opp_pos->x[0], 0.2, delta_t);
		opp_pos->y[0] = first_order_filter(last_opp_0[1], opp_pos->y[0], 0.2, delta_t);
		last_opp_0[0] = opp_pos->x[0];
		last_opp_0[1] = opp_pos->y[0];

	}
}

/*! \brief compute a single opponent position
*
* \param[in] last_rise last rise relative angle [rad]
* \param[in] last_fall last fall relative angle [rad]
* \param[in] rob_x robot x position [m]
* \param[in] rob_y robot y position [m]
* \param[in] rob_theta robot orientation [rad]
* \param[out] new_x_opp new known x opponent position [m]
* \param[out] new_y_opp new known y opponent position [m]
* \return 1 if computation successful, 0 otherwise
*/
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp)
{
	*new_x_opp = 0.0;
	*new_y_opp = 0.0;

	//correct theta to stay always positiv
	double dtheta = (last_fall - last_rise);
	if (dtheta < 0)
	{
		dtheta = dtheta + 2 * M_PI;
	}

	//compute theta to center of oponent
	double theta_rel = last_rise + dtheta / 2;

	//compute distance to oponent
	double distance = fabs(0.04 / sin(dtheta / 2));

	//calculate position of robot tower
	double xTower = rob_x + ROB_TOW*cos(rob_theta);
	double yTower = rob_y + ROB_TOW*sin(rob_theta);

	//calculate abs position of oponent
	double dxAbs = xTower + distance*cos(theta_rel + rob_theta);
	double dyAbs = yTower + distance*sin(theta_rel + rob_theta);

	*new_x_opp = dxAbs;
	*new_y_opp = dyAbs;


	return 1;
}

/*! \brief check if there is an opponent in front of the robot
*
*   Function not used, therefore not completed.
*
* \param[in] cvs controller main structure
* \return 1 if opponent robot in front of the current robot
*/
int check_opp_front(CtrlStruct *cvs)
{
	// variables declaration
	int i, nb_opp;

	OpponentsPosition *opp_pos;
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;
	nb_opp = opp_pos->nb_opp;

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

		// ----- opponents check computation end ----- //
	}

	return 0;
}

NAMESPACE_CLOSE();
