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

	static double last_opp_x0=0;
	static	double last_opp_y0=0;
	static double last_opp_x1 = 0;
	static	double last_opp_y1 = 0;

	CtrlIn *inputs;
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;

	// variables initialization
	inputs  = cvs->inputs;
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
		rise_index_2 = (rise_index_1-1 < 0) ? NB_STORE_EDGE-1 : rise_index_1-1;
		fall_index_2 = (fall_index_1-1 < 0) ? NB_STORE_EDGE-1 : fall_index_1-1;

		rise_2 = inputs->last_rising[rise_index_2];
		fall_2 = inputs->last_falling[fall_index_2];
	}

	// ----- opponents position computation start ----- //

	single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &(opp_pos->x[0]), &(opp_pos->y[0]));

	//std::cout << "Oponents position:\t" << opp_pos->x[0] << "\t" << opp_pos->y[0] << "\n";
	//std::cout << "Lponents position:\t" << last_opp_x0 << "\t" << last_opp_y0 << "\n";
	opp_pos->x[0] = first_order_filter(last_opp_x0, opp_pos->x[0], 0.2, delta_t);
	opp_pos->y[0] = first_order_filter(last_opp_y0, opp_pos->y[0], 0.2, delta_t);


	if(nb_opp==2){
		opp_pos->x[1] = 0.0;
		opp_pos->y[1] = 0.0;
	}
	// ----- opponents position computation end ----- //

	set_plot(opp_pos->x[0], "X-pos-oponent");
	set_plot(opp_pos->y[0], "Y-pos-OOponent");
	last_opp_x0 = opp_pos->x[0];
	last_opp_y0 = opp_pos->y[0];
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

	double dtheta = (last_fall - last_rise);
//	tan(dtheta) = dia_robo / distance;
	double distance = fabs(0.04 / sin(dtheta/2));
	double theta_rel = last_rise + dtheta / 2; ///todo change...
	//double drelX = distance*cos((dtheta / 2) + last_rise); //distance X from robot to oponent, tour-tour in robot x-y
	//double drelY = distance*sin((dtheta / 2) + last_rise);
	//std::cout << "dtheta" << dtheta*180/M_PI << "\n";
	//std::cout << "drelX: " << drelX << "\n";
	//std::cout << "drelY: " << drelY << "\n";
	double xTower = rob_x + ROB_TOW*cos(rob_theta);
	double yTower = rob_y + ROB_TOW*sin(rob_theta);

	//rotation to absolute x,y
	//double dxAbs = drelX + xTower*cos((dtheta / 2) + last_rise) - yTower*sin((dtheta / 2) + last_rise);
	//double dyAbs = drelY + xTower*sin((dtheta / 2) + last_rise) + yTower*cos((dtheta / 2) + last_rise);

	

	//direct
	double dxAbs = xTower + distance*cos(theta_rel + rob_theta);
	double dyAbs = xTower + distance*sin(theta_rel + rob_theta);

	*new_x_opp =  dxAbs;
	*new_y_opp =  dyAbs;
	

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

	for(i=0; i<nb_opp; i++)
	{
		// ----- opponents check computation start ----- //

		// ----- opponents check computation end ----- //
	}

	return 0;
}

NAMESPACE_CLOSE();
