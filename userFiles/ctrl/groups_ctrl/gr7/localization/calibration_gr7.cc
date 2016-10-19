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
enum {CALIB_START, CALIB_STATE_A, CALIB_STATE_B, CALIB_STATE_C, CALIB_STATE_D, CALIB_STATE_E, CALIB_FINISH};

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
	calib  = cvs->calib;

	rob_pos = cvs->rob_pos;
	
	t = inputs->t;
	team_id = cvs->team_id;

	// finite state machine (FSM)
	switch (calib->flag)
	{
		case CALIB_START: // start calibration
			speed_regulation(cvs, -20.0, -20.0);
			if (t>-14.0)
			{
				cvs->rob_pos->theta = -M_PI_2;
				cvs->rob_pos->y = (1.5 - 0.06); //1562 - distance du centre du robot
				calib->flag = CALIB_STATE_A; // directly go to state A
			}
			calib->t_flag = t;
			break;

		case CALIB_STATE_A: // state A
			speed_regulation(cvs, 20.0, 20.0);

			// go to state B after 1 seconds
			if (t - calib->t_flag > 0.5)
			{
				calib->flag = CALIB_STATE_B;

				calib->t_flag = t;
			}

			break;

		case CALIB_STATE_B: // state B
			speed_regulation(cvs, -10.0, 10.0);
	
			// go to state C after .7s
			if (t - calib->t_flag > 0.7)
			{
				calib->flag = CALIB_STATE_C;

				calib->t_flag = t;
			}
			break;

		case CALIB_STATE_C: // state C
			speed_regulation(cvs, -10.0, -10.0);

			// go to state D after 1.5 seconds
			if (t - calib->t_flag > 1.5)
			{
				cvs->rob_pos->x = (1.0-0.060);				//1.062-0.060
				calib->flag = CALIB_STATE_D;

				calib->t_flag = t;
			}
			break;
			//realignement
		case CALIB_STATE_D: // state C
			speed_regulation(cvs, 20.0, 20.0);

			// go to final state after 1 seconds
			if (t - calib->t_flag > 0.5)
			{
				calib->flag = CALIB_STATE_E;

				calib->t_flag = t;
			}
			break;		
		case CALIB_STATE_E: // state C
				speed_regulation(cvs, 10.0, -10.0);

				// go to final state after 2 seconds
				if (t - calib->t_flag > 0.7)
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

NAMESPACE_CLOSE();
