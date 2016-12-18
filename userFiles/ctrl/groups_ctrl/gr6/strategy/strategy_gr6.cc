#include "strategy_gr6.h"
#include "path_planning_gr6.h"
#include "speed_regulation_gr6.h"
#include "path_regulation_gr6.h"
#include "init_pos_gr6.h"
#include "opp_pos_gr6.h"
#include "odometry_gr6.h"
#include <math.h>
#include "TimeSensor.hh"
#include "kalman_gr6.h"
#include "obstacle_avoidance.hpp"

NAMESPACE_INIT(ctrlGr6);

/*! \brief intitialize the strategy structure
 * 
 * \return strategy structure initialized
 */
Strategy* init_strategy()
{
	Strategy *strat;
	strat = (Strategy*) malloc(sizeof(Strategy));
	strat->nb_target_before = 0;
	return strat;
}

/*! \brief release the strategy structure (memory released)
 * 
 * \param[out] strat strategy structure to release
 */
void free_strategy(Strategy *strat)
{
	free(strat);
}

/*! \brief startegy during the game
 * 
 * \param[in,out] cvs controller main structure
 */
void main_strategy(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	CtrlIn *inputs;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;
	PathPlanning* path = cvs->path;

	// update the taken value
	path_update_taken(cvs);

	// update the kalman filter
	kalman(cvs);

	// if we have 2 targets, but it was not planned, then go to base
	if (!strat->going_base && inputs->nb_targets == 2) {
		strat->main_state = GAME_STATE_DECIDE;
	}

	switch (strat->main_state)
	{
		case GAME_STATE_DECIDE:
			strat_decidePointToGo(cvs);
			break;

		case GAME_STATE_FIND_PATH:
			strat_findPath(cvs);
			break;

		case GAME_STATE_FOLLOW_PATH:
			strat_followPath(cvs);
			break;

		case GAME_STATE_AVOID:
			strat_avoid(cvs);
			break;

		case GAME_STATE_LAY_TARGET:
			strat_lay_target(cvs);
			break;

		case GAME_STATE_TAKE_TARGET:
			strat_take_target(cvs);
			break;

		case GAME_STATE_WIGGLE:
			strat_wiggle(cvs);
			break;

		case GAME_STATE_FINISH:
			strat_finish(cvs);
			break;
			
		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

// done
void strat_decidePointToGo(CtrlStruct* cvs)
{
	printf("strat_decidePointToGo\n");
	path_decision_making(cvs);
	return;
}

// done
void strat_findPath(CtrlStruct* cvs) {
	printf("strat_findPath : ");
	PathPlanning* path = cvs->path;
	Strategy* strat = cvs->strat;
	double d = 0;
	double kal_x = cvs->kalman->kalman_pos.x(), kal_y = cvs->kalman->kalman_pos.y();

	if (strat->going_base) {
		printf("base\n");
	}

	// find opt path
	path->nodeNumber = find_opt_path(strat->objectif_x, strat->objectif_y, kal_x, kal_y, cvs, &d);
	if (path->nodeNumber[0] == -1) {
		printf("strat_findPath : path not valid \n");
		system("PAUSE");
		exit(EXIT_FAILURE);
	}
	// create first node to go
	path->nextNode = path->nodeslist[path->nodeNumber[0]];

	// print the opt path index
	printf("opt path index : ");
	for (int i = 0; i < NB_NODE; i++) {
		if (path->nodeNumber[i] < 0) {
			printf("fin \n");
			break;
		}
		printf("%d ", path->nodeNumber[i]);
	}

	// new path -> reinit of path following variable
	path->path_created = true;
	path->path_finalNode = false;
	path->path_baseNode = false;
	path->arrivedAtNode = false;
	strat->main_state = GAME_STATE_FOLLOW_PATH;
	printf("strat_followPath\n");
	
	return;
}

// done
void strat_followPath(CtrlStruct* cvs) {
	//printf("strat_followPath\n");
	PathPlanning* path = cvs->path;
	Strategy* strat = cvs->strat;

	// if opposent in front, go to avoid state
	if (check_opp_front(cvs)) {
		strat->main_state = GAME_STATE_AVOID;
		printf("going into GAME_STATE_AVOID\n");
	}
	// if no opposent in front
	else {
		// check if we are at a node
		check_at_node(cvs);
		// if at a node
		if (path->arrivedAtNode) {
			// if final node
			if (path->path_finalNode) {
				// if at base node
				if (path->path_baseNode) {
					// then lay down the targets
					strat->main_state = GAME_STATE_LAY_TARGET;
					printf("going into GAME_STATE_LAY_TARGET\n");
				}
				// not a base_node -> node with a target
				else {
					// if on target, then take it
					if ((bool)cvs->inputs->target_detected) {
						strat->count = 0;
						strat->t_still = cvs->inputs->t;
						strat->main_state = GAME_STATE_TAKE_TARGET;
						printf("going into GAME_STATE_TAKE_TARGET\n");
					}
					// if not on a target but there should be a target
					else {
						strat->main_state = GAME_STATE_DECIDE;
						printf("going into GAME_STATE_DECIDE\n");
					}
				}
			}
			// if not a final node
			else {
				// not at final node -> change node to go
				path_nextNode(cvs);
				// stay in follow path mode
				strat->main_state = GAME_STATE_FOLLOW_PATH;
			}
		}
		// if not at a node, then move to node
		else {
			// move to objectif
			move_to_node(cvs);
			// stay in following path state
			strat->main_state = GAME_STATE_FOLLOW_PATH;
		}
	}

	return;
}

// todo
void strat_avoid(CtrlStruct* cvs) {
	//Initialization
	command_potential(cvs);
	float speed_right = cvs->path->command_v + cvs->path->command_w;
	float speed_left = cvs->path->command_v - cvs->path->command_w;
	
	speed_regulation(cvs, speed_right, speed_left);

	//speed_regulation(cvs, 0.0, 0.0);

	// if opposent is not in front, go to back to follow path state
	if (!check_opp_front(cvs)) {
		cvs->strat->main_state = GAME_STATE_FOLLOW_PATH;
		printf("going back to GAME_STATE_FOLLOW_PATH\n");
	}
	return;
}

// done
void strat_lay_target(CtrlStruct* cvs) {
	printf("strat_lay_target(s)\n");
	// release targets 
	cvs->outputs->flag_release = 1;
	// go to decision making state
	cvs->strat->main_state = GAME_STATE_DECIDE;
	return;
}

// done
void strat_take_target(CtrlStruct* cvs) {
	//printf("strat_take_target\n");
	// make sure we can take targets by make flag to 0 
	cvs->outputs->flag_release = 0;
	Strategy* strat = cvs->strat;
	CtrlIn* inputs = cvs->inputs;
	double t = cvs->inputs->t, t_still = strat->t_still;

	// stay still for T_STILL sec
	if ( t-t_still < TAKE_TARGET_T_STILL) {
		speed_regulation(cvs, 0.0, 0.0);
		// if we have more target than before trying to take the new one, target is taken, go to decision making
		if (inputs->nb_targets > strat->nb_target_before) {
			strat->main_state = GAME_STATE_DECIDE;
		}
	}
	else {
		// if we already tried TRY_MAX times to take the target, go to decide state
		if (strat->count > TAKE_TARGET_TRY_MAX) {
			strat->main_state = GAME_STATE_DECIDE;
		}
		// if we didnt already tried TRY_MAX times to take the target, move and try again
		else {
			strat->count++;
			strat->main_state = GAME_STATE_WIGGLE;
			strat->wiggle_t = inputs->t;
			strat->wiggle_state = MOVE_BACK;
		}
	}
		

	return;
}

// done but to check (it s a back up so may never be used)
void strat_wiggle(CtrlStruct* cvs) {
	printf("strat_wiggle\n");

	Strategy* strat = cvs->strat;
	CtrlIn* inputs = cvs->inputs;
	double t = inputs->t;
	int wiggle_state = strat->wiggle_state;

	// if time since last change bigger than WIGGLE_TIME sec,  unless last state of wiggling
	if (t - strat->wiggle_t > WIGGLE_TIME) {
		// if wiggling state is not the last one, go to the next state
		if (wiggle_state != MOVE_FRONT) {
			wiggle_state++;
			strat->wiggle_t = t;
		}
		// if wiggling state is the last one, go back to trying to take the target
		else {
			strat->main_state = GAME_STATE_TAKE_TARGET;
		}
		
	}

	// switch state for wiggling
	switch (wiggle_state) 
	{
		case MOVE_BACK:
			speed_regulation(cvs, -WIGGLE_SPEED, -WIGGLE_SPEED);
			break;
		case TURN:
			speed_regulation(cvs, -WIGGLE_SPEED, WIGGLE_SPEED);
			break;
		case MOVE_FRONT:
			speed_regulation(cvs, WIGGLE_SPEED, WIGGLE_SPEED);
			break;
		default:
			printf("error wiggle state\n");
	}

	return;
}

// done
void strat_finish(CtrlStruct* cvs) {
	printf("strat_finish\n");
	cvs->main_state = STOP_END_STATE;
	speed_regulation(cvs, 0.0, 0.0);
	return;
}

NAMESPACE_CLOSE();
