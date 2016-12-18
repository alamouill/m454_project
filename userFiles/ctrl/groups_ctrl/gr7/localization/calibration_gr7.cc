/* Mobile Robots Project - calibration_gr7.cc
 * Calibrate the position of the Robot
 * Version 1.2
 * Last Update: 18/12/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 


#include "calibration_gr7.h"
#include "speed_regulation_gr7.h"
#include "odometry_gr7.h"
#include "useful_gr7.h"
#include "init_pos_gr7.h"
#include <math.h>
#include <iostream>

NAMESPACE_INIT(ctrlGr7);

#define DEG_TO_RAD (M_PI/180.0) ///< convertion from degrees to radians

// calibration states
enum { CALIB_START, CALIB_STATE_A, CALIB_STATE_B, CALIB_STATE_C, CALIB_STATE_D, CALIB_STATE_E, CALIB_FINISH };

/*! \brief calibration of the robot to calibrate its position
*
* \param[in,out] cvs controller main structure
*
* This FSM can be adapted, depending on the map and on the robots initial position.
*/
void calibration(CtrlStruct *cvs)
{
	// variables declaration
	int team_id;
	double t;

	CtrlIn *inputs;
	RobotCalibration *calib;
	RobotPosition *rob_pos;

	// variables initialization
	inputs = cvs->inputs;
	calib = cvs->calib;

	rob_pos = cvs->rob_pos;

	t = inputs->t;
	team_id = cvs->team_id;
	// finite state machine (FSM)
	// team blue
	int k;
	if (team_id == 0)
	{
		k = 1;
	}
	else
	{
		k = -1;
	}
	/*switch (calib->flag)
	{
	case CALIB_START: // start calibration
	speed_regulation(cvs, -10.0, -10.0);
	//if both switchs are true wall is touched --> calibrating & go to state A
	if(inputs->u_switch[0] && inputs->u_switch[1])
	{
	// set the angle to -90 deg
	cvs->rob_pos->theta = -k*M_PI_2;
	// set the y position to
	cvs->rob_pos->y = k*(1.5 - 0.06); //1440 = p position of the wall - distance from center
	calib->flag = CALIB_STATE_A; // directly go to state A
	}
	calib->t_flag = t;
	break;

	case CALIB_STATE_A: // state A : get away from walls
	speed_regulation(cvs, 10.0, 10.0);

	// go to state B as soon as distance of 300mm from wall is achieved
	if (abs(cvs->rob_pos->y) <= 1.2)
	{
	calib->flag = CALIB_STATE_B;
	calib->t_flag = t;
	}

	break;

	case CALIB_STATE_B: // state B: turn to face perpendicular wall
	speed_regulation(cvs, -k*5.0, k*5.0);

	// go to state C if theta = ~PI
	if (cvs->rob_pos->theta <= k*M_PI + 0.005 && cvs->rob_pos->theta >= k*M_PI -0.005)
	{
	calib->flag = CALIB_STATE_C;

	calib->t_flag = t;
	}
	break;

	case CALIB_STATE_C: // state C go backwards into wall
	speed_regulation(cvs, -10.0, -10.0);

	// go to state D when both switches touch the wall
	if (inputs->u_switch[0] && inputs->u_switch[1])
	{
	cvs->rob_pos->x = (1.0-0.060);//0.940 = position of the wall - distance from center
	calib->flag = CALIB_STATE_D;

	calib->t_flag = t;
	}
	break;
	//realignement
	case CALIB_STATE_D: // state C
	speed_regulation(cvs, 10.0, 10.0);
	// go to final state if centered
	if (cvs->rob_pos->x <=0.750)
	{
	calib->flag = CALIB_STATE_E;
	calib->t_flag = t;
	}
	break;
	case CALIB_STATE_E: // state C
	speed_regulation(cvs, 10.0, -10.0);

	// go to final state if in direction of map
	if (cvs->rob_pos->theta >= -k*M_PI_2 -0.01 && cvs->rob_pos->theta <= -k*M_PI_2 +0.01)
	{
	calib->flag = CALIB_FINISH;
	cvs->main_state = WAIT_INIT_STATE;
	calib->t_flag = t;
	}
	break;
	case CALIB_FINISH: // wait for the match to start
	speed_regulation(cvs, 0.0, 0.0);
	break;

	default:
	printf("Error: unknown state : %d !\n", calib->flag);
	exit(EXIT_FAILURE);
	}

	*/

	if (team_id == 0)
	{
		switch (calib->flag)
		{
		case CALIB_START: // start calibration
			speed_regulation(cvs, -10.0, -10.0);
			//if both switchs are true wall is touched --> calibrating & go to state A
			if (inputs->u_switch[0] && inputs->u_switch[1])
			{
				// set the angle to -90 deg
				cvs->rob_pos->theta = -M_PI_2;
				// set the y position to 
				cvs->rob_pos->y = (1.5 - 0.06); //1440 = p position of the wall - distance from center
				calib->flag = CALIB_STATE_A; // directly go to state A
			}
			calib->t_flag = t;
			break;

		case CALIB_STATE_A: // state A : get away from walls
			speed_regulation(cvs, 10.0, 10.0);

			// go to state B as soon as distance of 300mm from wall is achieved
			if (cvs->rob_pos->y <= 1.2)
			{
				calib->flag = CALIB_STATE_B;
				calib->t_flag = t;
			}

			break;

		case CALIB_STATE_B: // state B: turn to face perpendicular wall
			speed_regulation(cvs, -5.0, 5.0);

			// go to state C if theta = ~PI 
			if (cvs->rob_pos->theta <= M_PI + 0.005 && cvs->rob_pos->theta >= M_PI - 0.005)
			{
				calib->flag = CALIB_STATE_C;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_C: // state C go backwards into wall
			speed_regulation(cvs, -10.0, -10.0);

			// go to state D when both switches touch the wall
			if (inputs->u_switch[0] && inputs->u_switch[1])
			{
				cvs->rob_pos->x = (1.0 - 0.060);//0.940 = position of the wall - distance from center
				calib->flag = CALIB_STATE_D;

				calib->t_flag = t;
			}
			break;
			//realignement
		case CALIB_STATE_D: // state C
			speed_regulation(cvs, 10.0, 10.0);
			// go to final state if centered
			if (cvs->rob_pos->x <= 0.750)
			{
				calib->flag = CALIB_STATE_E;
				calib->t_flag = t;
			}
			break;
		case CALIB_STATE_E: // state C
			speed_regulation(cvs, 10.0, -10.0);

			// go to final state if in direction of map 
			if (cvs->rob_pos->theta >= -M_PI_2 - 0.01 && cvs->rob_pos->theta <= -M_PI_2 + 0.01)
			{
				calib->flag = CALIB_FINISH;
				cvs->main_state = WAIT_INIT_STATE;
				calib->t_flag = t;
			}
			break;
		case CALIB_FINISH: // wait for the match to start
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown state : %d !\n", calib->flag);
			exit(EXIT_FAILURE);
		}
	}

	// team yellow
	else if (team_id == 1) {
		switch (calib->flag)
		{
		case CALIB_START: // start calibration
			speed_regulation(cvs, -10.0, -10.0);
			//if both switchs are true wall is touched --> calibrating & go to state A
			if (inputs->u_switch[0] && inputs->u_switch[1])
			{
				cvs->rob_pos->theta = M_PI_2;
				cvs->rob_pos->y = (-1.5 + 0.06); //1562 - distance du centre du robot
				calib->flag = CALIB_STATE_A; // directly go to state A
			}
			calib->t_flag = t;
			break;

		case CALIB_STATE_A: // state A : get away from walls
			speed_regulation(cvs, 10.0, 10.0);

			// go to state B as soon as distance of 300mm from wall is achieved
			if (cvs->rob_pos->y >= -1.2)
			{
				calib->flag = CALIB_STATE_B;

				calib->t_flag = t;
			}

			break;

		case CALIB_STATE_B: // state B: turns
			speed_regulation(cvs, 5.0, -5.0);

			// go to state C if theta = ~PI
			if (cvs->rob_pos->theta <= M_PI + 0.005 && cvs->rob_pos->theta >= M_PI - 0.005)
			{
				calib->flag = CALIB_STATE_C;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_C: // state C go backwards into wall
			speed_regulation(cvs, -10.0, -10.0);

			// go to state D when both switch touch the wall
			if (inputs->u_switch[0] && inputs->u_switch[1])
			{
				cvs->rob_pos->x = (1.0 - 0.060);				//1.062-0.060
				calib->flag = CALIB_STATE_D;

				calib->t_flag = t;
			}
			break;
			//realignement
		case CALIB_STATE_D: // state C
			speed_regulation(cvs, 10.0, 10.0);
			std::cout << "pos x: " << cvs->rob_pos->x << "\n";
			// go to final state if centered
			if (cvs->rob_pos->x <= 0.750)
			{
				calib->flag = CALIB_STATE_E;

				calib->t_flag = t;
			}
			break;
		case CALIB_STATE_E: // state C
			speed_regulation(cvs, -10.0, 10.0);

			// go to final state if in direction of map 
			if (cvs->rob_pos->theta >= M_PI_2 - 0.01 && cvs->rob_pos->theta <= M_PI_2 + 0.01)
			{
				calib->flag = CALIB_FINISH;
				cvs->main_state = WAIT_INIT_STATE;
				std::cout << "END OF CALIBRATION\n";
				calib->t_flag = t;
			}
			break;
		case CALIB_FINISH: // wait before the match is starting
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown state : %d !\n", calib->flag);
			exit(EXIT_FAILURE);
		}
	}

}

NAMESPACE_CLOSE();
