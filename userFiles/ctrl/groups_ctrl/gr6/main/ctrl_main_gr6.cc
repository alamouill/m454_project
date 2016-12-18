/*! 
 * \author Group 6
 * \file controller_main_gr6.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "ctrl_main_gr6.h"
#include "namespace_ctrl.h"
#include "init_pos_gr6.h"
#include "odometry_gr6.h"
#include "opp_pos_gr6.h"
#include "speed_regulation_gr6.h"
#include "calibration_gr6.h"
#include "triangulation_gr6.h"
#include "strategy_gr6.h"
#include "path_planning_gr6.h"
#include "path_regulation_gr6.h"
#include "kalman_gr6.h"

NAMESPACE_INIT(ctrlGr6);

/*! \brief initialize controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_init(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;

	// robot ID
	cvs->robot_id = inputs->robot_id;

	// robot team
	switch (inputs->robot_id)
	{
		case ROBOT_B: cvs->team_id = TEAM_A; break;
		case ROBOT_R: cvs->team_id = TEAM_A; break;
		case ROBOT_Y: cvs->team_id = TEAM_B; break;
		case ROBOT_W: cvs->team_id = TEAM_B; break;
	
		default:
			printf("Error: unknown robot ID: %d !\n", inputs->robot_id);
			exit(EXIT_FAILURE);
	}

	// number of opponents
	cvs->nb_opp = inputs->nb_opponents;

	// robot initial position
	set_init_position(cvs->robot_id, cvs->rob_pos);
	cvs->rob_pos->last_t = t;

	// speed regulation
	cvs->sp_reg->last_t = t;

	// calibration init
	calib_init(cvs);

	// if team B
	if (cvs->team_id == TEAM_B) {
		path_planning_correct_init(cvs);
	}
}

/*! \brief controller loop (called every time-step)
 * 
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{
	// variables declaration
	double t;
	CtrlIn *inputs;
	CtrlOut *outputs;
	RobotPosition *rob_pos;
	PathPlanning* path;
	float x = 0, y = 0, robx = 0, roby = 0, dx = 0, dy = 0;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	rob_pos = cvs->rob_pos;
	path = cvs->path;

	// time
	t = inputs->t;
	// update the robot odometry
	update_odometry(cvs);
	// triangulation
	triangulation(cvs);
	// opponents position
	opponents_tower(cvs);
	// tower control
	outputs->tower_command = 35.0;
		
	// opponent detection control
	if (0)
	{
		if (rob_pos->theta <-270)
			speed_regulation(cvs, 0, 0);
		else
			speed_regulation(cvs, -5, 5);
		return;
	}

	if (0)
	{
		speed_regulation(cvs, -5, 5);
		return;
	}
	
	// speed regulation control
	if (0)
	{
		if (t > -5)
			speed_regulation(cvs, -10, 10);
		else
			speed_regulation(cvs, 0, 0);

		// wheel speed
		//set_plot(inputs->l_wheel_speed, "input L wheel [ rad/ s ] ");
		//set_plot(inputs->r_wheel_speed, "input R wheel [ rad/ s ] ");

		// position
		//set_plot(rob_pos->x, "calc pos x");
		//set_plot(rob_pos->y, "calc pos y");
		//set_plot(rob_pos->theta, "calc theta");

		return;
	}

	// mvmt control
	if (0)
	{
		// simple control

		if (t > 0 && t < 5) {
			speed_regulation(cvs, 0, 0);
			//outputs->wheel_commands[R_ID] = 30;
			//outputs->wheel_commands[L_ID] = 30;
		}
		else if (t > 5 && t < 10) {
			speed_regulation(cvs, 10, -10);
			//outputs->wheel_commands[R_ID] = 30;
			//outputs->wheel_commands[L_ID] = -30;
		}
		else if (t > 10 && t < 15) {
			speed_regulation(cvs, 10, 5);
			//outputs->wheel_commands[R_ID] = 30;
			//outputs->wheel_commands[L_ID] = 15;
		}
		else {
			speed_regulation(cvs, 0, 0);
			//outputs->wheel_commands[R_ID] = 0;
			//outputs->wheel_commands[L_ID] = 0;
		}

		set_plot(inputs->l_wheel_speed, "input L wheel [ rad/ s ] ");
		set_plot(inputs->r_wheel_speed, "input R wheel [ rad/ s ] ");

		return;
	}
	
	
	switch (cvs->main_state)
	{
		// calibration
		case CALIB_STATE:
			calibration(cvs);
			break;

		// wait before match beginning
		case WAIT_INIT_STATE:
			speed_regulation(cvs, 0.0, 0.0);

			if (t > 0.0)
			{
				cvs->main_state = RUN_STATE;
				cvs->strat->main_state = GAME_STATE_DECIDE;
				correct_init_kalman(cvs);
			}
			break;

		// during game
		case RUN_STATE:
			
			main_strategy(cvs);
			if (t > 89.0) // 1 second safety
			{
				cvs->main_state = STOP_END_STATE;
				printf("end of game\n");
			}
			break;

		// stop at the end of the game
		case STOP_END_STATE:
			speed_regulation(cvs, 0.0, 0.0);
			outputs->flag_release = 1;
			break;

		case NB_MAIN_STATES:
			printf("Error: state NB_MAIN_STATES should not be reached !\n");
			exit(EXIT_FAILURE);
			break;
	
		default:
			printf("Error:unknown state : %d !\n", cvs->main_state);
			exit(EXIT_FAILURE);
	}
}

/*! \brief last controller operations (called once)
 * 
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{

}

NAMESPACE_CLOSE();
