/* Mobile Robots Project - opp_pos_gr7.cc
 * Measure Robot Position Using the beacon tower
 * Version 2.1
 * Last Update: 18/12/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 

#include "triangulation_gr7.h"
#include "useful_gr7.h"
#include "init_pos_gr7.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr7);

#define cot(x)      ( 1.0 / tan(x) )

/*! \brief set the fixed beacons positions, depending on the team
*
* \param[in] team_id ID of the team ('TEAM_A' or 'TEAM_B')
* \param[out] x_beac_1 first beacon x position [m]
* \param[out] y_beac_1 first beacon y position [m]
* \param[out] x_beac_2 second beacon x position [m]
* \param[out] y_beac_2 second beacon y position [m]
* \param[out] x_beac_3 third beacon x position [m]
* \param[out] y_beac_3 third beacon y position [m]
*
* This function can be adapted, depending on the map.
*/
void fixed_beacon_positions(int team_id, double *x_beac_1, double *y_beac_1,
	double *x_beac_2, double *y_beac_2, double *x_beac_3, double *y_beac_3)
{
	switch (team_id)
	{
	case TEAM_A:
		*x_beac_1 = -1.062;
		*y_beac_1 = 1.562;

		*x_beac_2 = 1.062;
		*y_beac_2 = 1.562;

		*x_beac_3 = 0.0;
		*y_beac_3 = -1.562;
		break;

	case TEAM_B:
		*x_beac_1 = 0;
		*y_beac_1 = 1.562;

		*x_beac_2 = 1.062;
		*y_beac_2 = -1.562;

		*x_beac_3 = -1.062;
		*y_beac_3 = -1.562;
		break;

	default:
		printf("Error unknown team ID (%d) !\n", team_id);
		exit(EXIT_FAILURE);
	}
}

/*! \brief get the index of the best angle prediction
*
* \param[in] alpha_predicted angle to reach [rad]
* \param[in] alpha_a angle computed for A [rad]
* \param[in] alpha_b angle computed for B [rad]
* \param[in] alpha_c angle computed for C [rad]
* \return best index (0, 1, or 2)
*/
int index_predicted(double alpha_predicted, double alpha_a, double alpha_b, double alpha_c)
{
	double pred_err_a, pred_err_b, pred_err_c;

	pred_err_a = fabs(limit_angle(alpha_a - alpha_predicted));
	pred_err_b = fabs(limit_angle(alpha_b - alpha_predicted));
	pred_err_c = fabs(limit_angle(alpha_c - alpha_predicted));

	return (pred_err_a < pred_err_b) ? ((pred_err_a < pred_err_c) ? 0 : 2) : ((pred_err_b < pred_err_c) ? 1 : 2);
}

/*! \brief calculates the angle given by the odometry to intialize the triangulation
*
* \param[in] x beacon position
* \param[in] y beacon position
* \param[in] robot position
* \return angle estimated by odometry
*/
static double alpha_odo(double x_beac, double y_beac, RobotPosition* rob_pos)
{
	double alpha = atan2(y_beac - rob_pos->y, x_beac - rob_pos->x) - atan2(sin(rob_pos->theta), cos(rob_pos->theta));
	if (fabs(alpha) > M_PI)
		alpha -= sgn(alpha) * 2 * M_PI;
	return alpha;
}
/*! \brief computes the angle between two successive beacons
* \param[in] last fall time
* \param[in] last rise time
* \return angle between the two beacons
*/
static double alpha_laser(double alpha_falling, double alpha_rising)
{
	double alpha;
	if ((alpha_rising > 0) && (alpha_falling < 0))
		alpha_falling += 2 * M_PI;
	alpha = (alpha_rising + alpha_falling) / 2;
	if (fabs(alpha) > M_PI)
		alpha -= sgn(alpha) * 2 * M_PI;
	return alpha;
}
/*! \brief triangulation main algorithm
*
* \param[in] cvs controller main structure
*
* computation found here: http://www.telecom.ulg.ac.be/triangulation/
*/
void triangulation(CtrlStruct *cvs)
{
	// variables declaration
	RobotPosition *pos_tri, *rob_pos;
	CtrlIn *inputs;

	int alpha_1_index, alpha_2_index, alpha_3_index;
	int rise_index_1, rise_index_2, rise_index_3;
	int fall_index_1, fall_index_2, fall_index_3;

	double alpha_a, alpha_b, alpha_c;
	double alpha_1, alpha_2, alpha_3;
	double alpha_1_predicted, alpha_2_predicted, alpha_3_predicted;
	double x_beac_1, y_beac_1, x_beac_2, y_beac_2, x_beac_3, y_beac_3;

	// variables initialization
	pos_tri = cvs->triang_pos;
	rob_pos = cvs->rob_pos;
	inputs = cvs->inputs;

	// safety
	if ((inputs->rising_index_fixed < 0) || (inputs->falling_index_fixed < 0))
	{
		return;
	}

	// known positions of the beacons
	fixed_beacon_positions(cvs->team_id, &x_beac_1, &y_beac_1, &x_beac_2, &y_beac_2, &x_beac_3, &y_beac_3);

	// indexes fot the angles detection
	rise_index_1 = inputs->rising_index_fixed;
	rise_index_2 = (rise_index_1 - 1 < 0) ? NB_STORE_EDGE - 1 : rise_index_1 - 1;
	rise_index_3 = (rise_index_2 - 1 < 0) ? NB_STORE_EDGE - 1 : rise_index_2 - 1;

	fall_index_1 = inputs->falling_index_fixed;
	fall_index_2 = (fall_index_1 - 1 < 0) ? NB_STORE_EDGE - 1 : fall_index_1 - 1;
	fall_index_3 = (fall_index_2 - 1 < 0) ? NB_STORE_EDGE - 1 : fall_index_2 - 1;

	// beacons angles measured with the laser
	alpha_a = alpha_laser(inputs->last_falling_fixed[fall_index_1], inputs->last_rising_fixed[fall_index_1]);
	alpha_b = alpha_laser(inputs->last_falling_fixed[fall_index_2], inputs->last_rising_fixed[fall_index_2]);
	alpha_c = alpha_laser(inputs->last_falling_fixed[fall_index_3], inputs->last_rising_fixed[fall_index_3]);



	// beacons angles predicted thanks to odometry measurements (to compute)
	alpha_1_predicted = alpha_odo(x_beac_1, y_beac_1, rob_pos);
	alpha_2_predicted = alpha_odo(x_beac_2, y_beac_2, rob_pos);
	alpha_3_predicted = alpha_odo(x_beac_3, y_beac_3, rob_pos);

	//printf("%f %f\n", inputs->last_falling_fixed[fall_index_1], inputs->last_rising_fixed[fall_index_1]);
	//set_plot(alpha_1_predicted, "a1p");
	//set_plot(alpha_2_predicted, "a2p");
	//set_plot(alpha_3_predicted, "a3p");

	/*alpha_a = alpha_1_predicted;
	alpha_b = alpha_2_predicted;
	alpha_c = alpha_3_predicted;*/

	// indexes of each beacon
	alpha_1_index = index_predicted(alpha_1_predicted, alpha_a, alpha_b, alpha_c);
	alpha_2_index = index_predicted(alpha_2_predicted, alpha_a, alpha_b, alpha_c);
	alpha_3_index = index_predicted(alpha_3_predicted, alpha_a, alpha_b, alpha_c);
	//printf("coucou\n");
	// safety
	//printf("%f %f %f %f %f %f\n", alpha_a, alpha_1_predicted, alpha_b, alpha_2_predicted, alpha_c, alpha_3_predicted);
	if ((alpha_1_index == alpha_2_index) || (alpha_1_index == alpha_3_index) || (alpha_2_index == alpha_3_index))
	{
		//printf("%f %f %f %f %f %f\n", alpha_a, alpha_1_predicted, alpha_b, alpha_2_predicted, alpha_c, alpha_3_predicted);
		return;
	}
	//printf("coucou2\n");
	// angle of the first beacon
	switch (alpha_1_index)
	{
	case 0: alpha_1 = alpha_a; break;
	case 1: alpha_1 = alpha_b; break;
	case 2: alpha_1 = alpha_c; break;

	default:
		printf("Error: unknown index %d !\n", alpha_1_index);
		exit(EXIT_FAILURE);
	}

	// angle of the second beacon
	switch (alpha_2_index)
	{
	case 0: alpha_2 = alpha_a; break;
	case 1: alpha_2 = alpha_b; break;
	case 2: alpha_2 = alpha_c; break;

	default:
		printf("Error: unknown index %d !\n", alpha_2_index);
		exit(EXIT_FAILURE);
	}

	// angle of the third beacon
	switch (alpha_3_index)
	{
	case 0: alpha_3 = alpha_a; break;
	case 1: alpha_3 = alpha_b; break;
	case 2: alpha_3 = alpha_c; break;

	default:
		printf("Error: unknown index %d !\n", alpha_3_index);
		exit(EXIT_FAILURE);
	}


	/*alpha_1 = alpha_1_predicted;
	alpha_2 = alpha_2_predicted;
	alpha_3 = alpha_3_predicted;*/

	// ----- triangulation computation start ----- //

	double alpha_12 = alpha_2 - alpha_1;
	double alpha_23 = alpha_3 - alpha_2;
	double alpha_31 = alpha_1 - alpha_3;

	//printf("%f \n", rob_pos->theta);
	//printf("%f %f %f\n", alpha_12, alpha_23, alpha_31);
	/*set_plot(alpha_1, "a12");
	set_plot(alpha_2, "a23");
	set_plot(alpha_3, "a31");*/

	static double last_t = -15;

	double x1_, y1_, x2_, y2_, x3_, y3_;

	double cot_12, cot_23, cot_31, c12x, c12y, c23x, c23y, c31x, c31y, k, D, invD, K;

	//set_plot(pos_tri->theta, "theta triang");

	if (fabs(alpha_12 - M_PI)<0.00001 || alpha_12 == 0.0)
	{
		//printf("coucou\n");
		x1_ = x_beac_1 - x_beac_3, y1_ = y_beac_1 - y_beac_3, x2_ = x_beac_2 - x_beac_3, y2_ = y_beac_2 - y_beac_3;

		cot_23 = cot(alpha_23);

		c12x = (y1_ - y2_);
		c12y = (x2_ - x1_);
		k = (y1_ * x2_) - (x1_ * y2_);

		c23x = x2_ + cot_23 * y2_;
		c23y = y2_ - cot_23 * x2_;

		c31x = x1_ + cot_23 * y1_;
		c31y = y1_ - cot_23 * x1_;

		D = (c31x - c23x) * (c12y)+(c12x) * (c23y - c31y);
		invD = 1.0 / D;
		K = k * invD;

		pos_tri->x = first_order_filter(pos_tri->x, K * (c23y - c31y) + x_beac_3 - 0.083 * cos(rob_pos->theta), 0.1, inputs->t - last_t);
		pos_tri->y = first_order_filter(pos_tri->y, K * (c31x - c23x) + y_beac_3 - 0.083 * sin(rob_pos->theta), 0.1, inputs->t - last_t);
		pos_tri->theta = atan2(pos_tri->y - y_beac_1, pos_tri->x - x_beac_1) - alpha_1 - M_PI;
		pos_tri->theta = limit_angle(pos_tri->theta);

		return;
	}

	if (fabs(alpha_23 - M_PI) < 0.00001 || alpha_23 == 0.0)
	{
		//printf("coucou 22222\n");
		x2_ = x_beac_2 - x_beac_1, y2_ = y_beac_2 - y_beac_1, x3_ = x_beac_3 - x_beac_1, y3_ = y_beac_3 - y_beac_1;

		cot_31 = cot(alpha_31);

		c12x = x2_ + cot_31 * y2_;
		c12y = y2_ - cot_31 * x2_;

		c23x = (y2_ - y3_);
		c23y = (x3_ - x2_);
		k = (y2_ * x3_) - (x2_ * y3_);

		c31x = x3_ + cot_31 * y3_;
		c31y = y3_ - cot_31 * x3_;

		D = (c12x - c31x) * (c23y)+(c23x) * (c31y - c12y);
		invD = 1.0 / D;
		K = k * invD;

		pos_tri->x = first_order_filter(pos_tri->x, K * (c31y - c12y) + x_beac_1 - 0.083 * cos(rob_pos->theta), 0.1, inputs->t - last_t);
		pos_tri->y = first_order_filter(pos_tri->y, K * (c12x - c31x) + y_beac_1 - 0.083 * sin(rob_pos->theta), 0.1, inputs->t - last_t);
		pos_tri->theta = atan2(pos_tri->y - y_beac_1, pos_tri->x - x_beac_1) - alpha_1 - M_PI;
		pos_tri->theta = limit_angle(pos_tri->theta);

		return;
	}

	if (fabs(alpha_31 - M_PI) < 0.00001 || alpha_31 == 0.0)
	{
		//printf("coucou 3333333333\n");
		x1_ = x_beac_1 - x_beac_2, y1_ = y_beac_1 - y_beac_2, x3_ = x_beac_3 - x_beac_2, y3_ = y_beac_3 - y_beac_2;

		cot_12 = cot(alpha_12);

		c12x = x1_ + cot_12 * y1_;
		c12y = y1_ - cot_12 * x1_;

		c23x = x3_ + cot_12 * y3_;
		c23y = y3_ - cot_12 * x3_;

		c31x = (y3_ - y1_);
		c31y = (x1_ - x3_);
		k = (y3_ * x1_) - (x3_ * y1_);

		D = (c23x - c12x) * (c31y)+(c31x) * (c12y - c23y);
		invD = 1.0 / D;
		K = k * invD;

		pos_tri->x = first_order_filter(pos_tri->x, K * (c12y - c23y) + x_beac_2 - 0.083 * cos(rob_pos->theta), 0.1, inputs->t - last_t);
		pos_tri->y = first_order_filter(pos_tri->y, K * (c23x - c12x) + y_beac_2 - 0.083 * sin(rob_pos->theta), 0.1, inputs->t - last_t);
		pos_tri->theta = atan2(pos_tri->y - y_beac_1, pos_tri->x - x_beac_1) - alpha_1 - M_PI;
		pos_tri->theta = limit_angle(pos_tri->theta);

		return;
	}

	x1_ = x_beac_1 - x_beac_2, y1_ = y_beac_1 - y_beac_2, x3_ = x_beac_3 - x_beac_2, y3_ = y_beac_3 - y_beac_2;
	//printf("coucou44444444444444444\n");
	cot_12 = cot(alpha_12);
	cot_23 = cot(alpha_23);
	cot_31 = (1.0 - cot_12*cot_23) / (cot_12 + cot_23);

	c12x = x1_ + cot_12 * y1_;
	c12y = y1_ - cot_12 * x1_;

	c23x = x3_ - cot_23 * y3_;
	c23y = y3_ + cot_23 * x3_;

	c31x = (x3_ + x1_) + cot_31 * (y3_ - y1_);
	c31y = (y3_ + y1_) - cot_31 * (x3_ - x1_);
	k = (x3_ * x1_) + (y3_ * y1_) + cot_31 * ((y3_ * x1_) - (x3_ * y1_));

	D = (c12x - c23x) * (c23y - c31y) - (c23x - c31x) * (c12y - c23y);
	invD = 1.0 / D;
	K = k * invD;
	pos_tri->x = first_order_filter(pos_tri->x, K * (c12y - c23y) + x_beac_2 - 0.083 * cos(rob_pos->theta), 0.1, inputs->t - last_t);
	pos_tri->y = first_order_filter(pos_tri->y, K * (c23x - c12x) + y_beac_2 - 0.083 * sin(rob_pos->theta), 0.1, inputs->t - last_t);
	pos_tri->theta = atan2(pos_tri->y - y_beac_1, pos_tri->x - x_beac_1) - alpha_1 - M_PI;
	pos_tri->theta = limit_angle(pos_tri->theta);
	last_t = inputs->t;
	// ----- triangulation computation end ----- //
}

NAMESPACE_CLOSE();
