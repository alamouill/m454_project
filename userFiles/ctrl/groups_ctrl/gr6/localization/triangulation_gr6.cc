#include "triangulation_gr6.h"
#include "useful_gr6.h"
#include "init_pos_gr6.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr6);

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

		*x_beac_2 = 0;
		*y_beac_2 = -1.562;

		*x_beac_3 = 1.062;
		*y_beac_3 = 1.562;
		break;

	case TEAM_B:
		*x_beac_1 = 1.062;
		*y_beac_1 = -1.562;

		*x_beac_2 = 0;
		*y_beac_2 = 1.562;

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

	double delta_t = 0;
	double tau = 0.1;
	double prev_x = 0, prev_y = 0;

	static int err=0, pas_err = 0;
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

	prev_x = pos_tri->x;
	prev_y = pos_tri->y;
	delta_t = inputs->t - pos_tri->last_t;
	pos_tri->last_t = inputs->t;

	// safety
	if ((inputs->rising_index_fixed < 0) || (inputs->falling_index_fixed < 0))
	{
		return;
	}

	// known positions of the beacons
	fixed_beacon_positions(cvs->team_id, &x_beac_1, &y_beac_1, &x_beac_2, &y_beac_2, &x_beac_3, &y_beac_3);

	// indexes for the angles detection
	rise_index_1 = inputs->rising_index_fixed;
	rise_index_2 = (rise_index_1 - 1 < 0) ? NB_STORE_EDGE - 1 : rise_index_1 - 1;
	rise_index_3 = (rise_index_2 - 1 < 0) ? NB_STORE_EDGE - 1 : rise_index_2 - 1;

	fall_index_1 = inputs->falling_index_fixed;
	fall_index_2 = (fall_index_1 - 1 < 0) ? NB_STORE_EDGE - 1 : fall_index_1 - 1;
	fall_index_3 = (fall_index_2 - 1 < 0) ? NB_STORE_EDGE - 1 : fall_index_2 - 1;

	alpha_a = (inputs->last_falling_fixed[fall_index_1] - inputs->last_rising_fixed[rise_index_1]) / 2 + inputs->last_rising_fixed[rise_index_1]; // rad
	alpha_b = (inputs->last_falling_fixed[fall_index_2] - inputs->last_rising_fixed[rise_index_2]) / 2 + inputs->last_rising_fixed[rise_index_2]; // rad
	alpha_c = (inputs->last_falling_fixed[fall_index_3] - inputs->last_rising_fixed[rise_index_3]) / 2 + inputs->last_rising_fixed[rise_index_3]; // rad

	// beacons angles predicted thanks to odometry measurements
	double x1 = x_beac_1 - rob_pos->x + DIST_ROB_TOW*cos((rob_pos->theta)*DEG_TO_RAD);
	double y1 = y_beac_1 - rob_pos->y + DIST_ROB_TOW*sin((rob_pos->theta)*DEG_TO_RAD);

	double x2 = x_beac_2 - rob_pos->x + DIST_ROB_TOW*cos((rob_pos->theta)*DEG_TO_RAD);
	double y2 = y_beac_2 - rob_pos->y + DIST_ROB_TOW*sin((rob_pos->theta)*DEG_TO_RAD);

	double x3 = x_beac_3 - rob_pos->x + DIST_ROB_TOW*cos((rob_pos->theta)*DEG_TO_RAD);
	double y3 = y_beac_3 - rob_pos->y + DIST_ROB_TOW*sin((rob_pos->theta)*DEG_TO_RAD);

	alpha_1_predicted = atan2(y1, x1) - rob_pos->theta*DEG_TO_RAD; // rad
	alpha_2_predicted = atan2(y2, x2) - rob_pos->theta*DEG_TO_RAD; // rad
	alpha_3_predicted = atan2(y3, x3) - rob_pos->theta*DEG_TO_RAD; // rad
	
	// get the predicted angles between -pi and pi
	alpha_1_predicted = limit_angle(alpha_1_predicted);
	alpha_2_predicted = limit_angle(alpha_2_predicted);
	alpha_3_predicted = limit_angle(alpha_3_predicted);

	// indexes of each beacon
	alpha_1_index = index_predicted(alpha_1_predicted, alpha_a, alpha_b, alpha_c);
	alpha_2_index = index_predicted(alpha_2_predicted, alpha_a, alpha_b, alpha_c);
	alpha_3_index = index_predicted(alpha_3_predicted, alpha_a, alpha_b, alpha_c);

	// safety
	static int i = 0;
	if ((alpha_1_index == alpha_2_index) || (alpha_1_index == alpha_3_index) || (alpha_2_index == alpha_3_index))
	{
		return;
	}

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

	// ----- triangulation computation start ----- //

	// robot position
	pos_tri->x = 0.0;
	pos_tri->y = 0.0;

	// robot orientation
	pos_tri->theta = 0.0;

	//compute the modified beacon coordinates
	float x1_p = x_beac_1 - x_beac_2;
	float y1_p = y_beac_1 - y_beac_2;
	float x3_p = x_beac_3 - x_beac_2;
	float y3_p = y_beac_3 - y_beac_2;

	//compute the three cot();
	float T12 = 1 / (tan(alpha_2 - alpha_1));
	float T23 = 1 / (tan(alpha_3 - alpha_2));
	float T31 = (1 - T12*T23) / (T12 + T23);

	//compute the modified circle center coordinates
	float x12_p = x1_p + T12*y1_p, y12_p = y1_p - T12*x1_p;
	float x23_p = x3_p - T23*y3_p, y23_p = y3_p + T23*x3_p;
	float x31_p = x3_p + x1_p + T31*(y3_p - y1_p);
	float y31_p = y3_p + y1_p - T31*(x3_p - x1_p);

	//compute k31_p
	float k31_p = x1_p*x3_p + y1_p*y3_p + T31*(x1_p*y3_p - x3_p*y1_p);

	//compute D
	float D = (x12_p - x23_p)*(y23_p - y31_p) - (y12_p - y23_p)*(x23_p - x31_p);
	if (D == 0)
	{
		printf("ERROR D=0");
		return;
	}

	// compute the tower position
	pos_tri->x = (x_beac_2 + k31_p*(y12_p - y23_p) / D);
	pos_tri->y = (y_beac_2 + k31_p*(x23_p - x12_p) / D);

	// compute the robot position
	pos_tri->x = pos_tri->x - DIST_ROB_TOW*cos(rob_pos->theta*DEG_TO_RAD);
	pos_tri->y = pos_tri->y - DIST_ROB_TOW*sin(rob_pos->theta*DEG_TO_RAD);

	// filter the position
	pos_tri->x = first_order_filter(prev_x, pos_tri->x, tau, delta_t);
	pos_tri->y = first_order_filter(prev_y, pos_tri->y, tau, delta_t);

	//set_plot(pos_tri->x, "x_tri [m]");
	//set_plot(pos_tri->y, "y_tri [m]");

	// ----- triangulation computation end ----- //
	return;
}

NAMESPACE_CLOSE();