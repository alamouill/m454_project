#include "calibration_gr6.h"
#include "speed_regulation_gr6.h"
#include "odometry_gr6.h"
#include "useful_gr6.h"
#include "init_pos_gr6.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr6);

#define DEG_TO_RAD (M_PI/180.0) ///< convertion from degrees to radians

// calibration states
enum { CALIB_START, CALIB_STATE_A, CALIB_STATE_B, CALIB_STATE_C, CALIB_FINISH };

// calibration value
#define TEAM_A_POSX 0.560
#define TEAM_A_POSY 1.440
#define TEAM_A_THETA1 -90
#define TEAM_A_THETA2 0
#define TEAM_A_FINAL_THETA -90
#define TEAM_A_ROT_R_SP 6
#define TEAM_A_ROT_L_SP 2


#define TEAM_B_POSX 0.560
#define TEAM_B_POSY -1.440
#define TEAM_B_THETA1 90
#define TEAM_B_THETA2 0
#define TEAM_B_FINAL_THETA 90
#define TEAM_B_ROT_R_SP 2
#define TEAM_B_ROT_L_SP 6


void calib_init(CtrlStruct *cvs) {
	RobotCalibration *calib;
	calib = cvs->calib;
	if (cvs->team_id == TEAM_A) {
		calib->pos_x = TEAM_A_POSX;
		calib->pos_y = TEAM_A_POSY;
		calib->theta1 = TEAM_A_THETA1;
		calib->theta2 = TEAM_A_THETA2;
		calib->final_theta = TEAM_A_FINAL_THETA;
		calib->rot_speed_r = TEAM_A_ROT_R_SP;
		calib->rot_speed_l = TEAM_A_ROT_L_SP;
	}
	else {
		calib->pos_x = TEAM_B_POSX;
		calib->pos_y = TEAM_B_POSY;
		calib->theta1 = TEAM_B_THETA1;
		calib->theta2 = TEAM_B_THETA2;
		calib->final_theta = TEAM_B_FINAL_THETA;
		calib->rot_speed_r = TEAM_B_ROT_R_SP;
		calib->rot_speed_l = TEAM_B_ROT_L_SP;
	}
}

/*! \brief calibration of the robot to calibrate its position
*
* \param[in,out] cvs controller main structure
*
* This FSM can be adapted, depending on the map and on the robots initial position.
*/
void calibration(CtrlStruct *cvs)
{
	// variables declaration
	double t;

	CtrlIn *inputs;
	RobotCalibration *calib;
	RobotPosition *rob_pos;

	// variables initialization
	inputs = cvs->inputs;
	calib = cvs->calib;

	rob_pos = cvs->rob_pos;
	t = inputs->t;

	// finite state machine (FSM)
	switch (calib->flag)
	{
	case CALIB_START: // start calibration
		// if no switch touch the wall
		if (inputs->u_switch[R_ID] == 0 && inputs->u_switch[L_ID] == 0) {
			// move backward
			speed_regulation(cvs, -10, -10);
		}
		// if right switch touch wall, turn backward left
		else if (inputs->u_switch[R_ID] == 0) {
			speed_regulation(cvs, -5, 0);
		}
		// if left switch touch wall, turn backward right
		else if (inputs->u_switch[L_ID] == 0) {
			speed_regulation(cvs, 0, -5);
		}
		
		// when hit the wall
		if (inputs->u_switch[R_ID] == 1 && inputs->u_switch[L_ID] == 1 && (t>-13.6))
		{
			speed_regulation(cvs, 0, 0);
			rob_pos->y = calib->pos_y;
			rob_pos->theta = calib->theta1;

			calib->flag = CALIB_STATE_A; // go to state A
			calib->t_flag = t;
		}
		break;

	case CALIB_STATE_A: // state A
		// stay still for 0.5 s then start turning until 3.5
		if ((t - calib->t_flag > 0.5) && (t - calib->t_flag < 3.5))
		{
			
			speed_regulation(cvs,calib->rot_speed_r, calib->rot_speed_l);
		}
		else
		{
			speed_regulation(cvs, 0.0, 0.0);
		}
		// go to state C after 4 seconds in state A
		if (t - calib->t_flag > 4.0)
		{
			calib->flag = CALIB_STATE_B;
			calib->t_flag = t;
		}
		break;

	case CALIB_STATE_B: // state B
		// if no switch touch the wall
		if (inputs->u_switch[R_ID] == 0 && inputs->u_switch[L_ID] == 0) {
			// move backward
			speed_regulation(cvs, -10, -10);
		}
		// if right switch touch wall, turn backward left
		else if (inputs->u_switch[R_ID] == 0) {
			speed_regulation(cvs, -5, 0);
		}
		// if left switch touch wall, turn backward right
		else if (inputs->u_switch[L_ID] == 0) {
			speed_regulation(cvs, 0, -5);
		}

		// when hit the wall
		if (inputs->u_switch[R_ID] == 1 && inputs->u_switch[L_ID] == 1 && (t-calib->t_flag>1.5))
		{
			speed_regulation(cvs, 0.0, 0.0);
			rob_pos->theta = calib->theta2;
			rob_pos->x = calib->pos_x;

			calib->flag = CALIB_STATE_C;
			calib->t_flag = t;
		}
		break;

	case CALIB_STATE_C: // state C
		// wait 0.5 s then turn until facing right direction
		if (!(calib->final_theta - 1 < rob_pos->theta && rob_pos->theta < calib->final_theta + 1) && (t - calib->t_flag > 0.5)) {
			speed_regulation(cvs, calib->rot_speed_l, calib->rot_speed_r);
		}
		else
		{
			if ((t - calib->t_flag > 5) && (t - calib->t_flag < 7))
			{
				speed_regulation(cvs, -5, -5);
			}
			else
			{
				speed_regulation(cvs, 0, 0);
			}
		}
		if (t - calib->t_flag > 7)
		{
			calib->flag = CALIB_FINISH;
			calib->t_flag = t;
		}
		break;

	case CALIB_FINISH: // wait before the match is starting
		cvs->main_state = WAIT_INIT_STATE;
		break;
	default:
		printf("Error: unknown state : %d !\n", calib->flag);
		exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
