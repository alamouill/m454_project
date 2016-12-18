#include "strategy_gr7.h"
#include "path_planning_gr7.h"
#include "speed_regulation_gr7.h"
#include "path_regulation_gr7.h"
#include "init_pos_gr7.h"
#include "opp_pos_gr7.h"
#include "odometry_gr7.h"
#include <math.h>
#include <iostream>
#include "useful_gr7.h"

NAMESPACE_INIT(ctrlGr7);

#define capturingDetectionCoeff 3
#define NB_TARGETS 8
#define OPP_CAPT_R 100 //[mm]
#define KN	5
#define KD	1.01
#define RETURNTIME 80


/*! \brief intitialize the strategy structure
*
* \return strategy structure initialized
*/
Strategy* init_strategy()
{
	Strategy *strat;

	strat = (Strategy*)malloc(sizeof(Strategy));

	// Coordinates + score of the goal
	//			X									Y								Score
	strat->coord_goal[0][0] = -800;		strat->coord_goal[0][1] = 0;		strat->coord_goal[0][2] = 3;
	strat->coord_goal[1][0] = 250;		strat->coord_goal[1][1] = 1250;		strat->coord_goal[1][2] = 2;
	strat->coord_goal[2][0] = 100;		strat->coord_goal[2][1] = 0;		strat->coord_goal[2][2] = 2;
	strat->coord_goal[3][0] = 250;		strat->coord_goal[3][1] = -1250;	strat->coord_goal[3][2] = 2;
	strat->coord_goal[4][0] = 700;		strat->coord_goal[4][1] = -600;		strat->coord_goal[4][2] = 1;
	strat->coord_goal[5][0] = -400;		strat->coord_goal[5][1] = -600;		strat->coord_goal[5][2] = 1;
	strat->coord_goal[6][0] = -400;		strat->coord_goal[6][1] = 600;		strat->coord_goal[6][2] = 1;
	strat->coord_goal[7][0] = 700;		strat->coord_goal[7][1] = 600;		strat->coord_goal[7][2] = 1;
	strat->index_goal = 0;

	for (int i = 0; i < NB_TARGETS; i++) {
		strat->coord_goal[i][OPP_CAP_STATE] = empty;
		strat->avilability[i] = 100;
	}


	return strat;
}



/*! \brief evaluates the oponents behaviour.
*
* \modifies the goals evaluation
*/
void evaluate_Oponent(CtrlStruct *cvs) {
	/*	static double goals[3][4] = {
	//coords, coords, status, taken %
	{ { 0.700 },{ -0.600 },1,100 },
	{ { 0.100 },{ 0 },1,100 },
	{ { -0.400 },{ 0.600 },1,100 }
	};*/
	//	cvs->strat->coord_goal
	static double lastT;
	int NBOPPS = 1;
	double oppPos[2];
	for (int i = 0; i < NB_TARGETS; i++) {
		for (int j = 0; j < NBOPPS; j++) {
			oppPos[0] = cvs->opp_pos->x[j] * 1000;		//in [mm]
			oppPos[1] = cvs->opp_pos->y[j] * 1000;		//in [mm]
			double goal[] = {
				cvs->strat->coord_goal[i][X],			// in [mm]
				cvs->strat->coord_goal[i][Y], };			// in [mm]

			if (get_Distance(oppPos, goal) < OPP_CAPT_R) {
				//		std::cout << "_evaluate pos/goal " << oppPos[0] << "/" << oppPos[1] << "\t" << goal[0] << "\t" << goal[1] << " for i= " << i << "\n";

				if (cvs->strat->coord_goal[i][OPP_CAP_STATE] == empty) {
					cvs->strat->coord_goal[i][OPP_CAP_STATE] = capturing;
					//				goals[i][3] = cvs->inputs->t; //set timer to momentary time
				}
				else if (cvs->strat->coord_goal[i][OPP_CAP_STATE] == capturingCheck) {
					cvs->strat->coord_goal[i][OPP_CAP_STATE] = capturingCheckOk;
				}


			}
		}
		//capturingCheck not fulfilled, goal is resetted
		if (cvs->strat->coord_goal[i][OPP_CAP_STATE] == capturingCheck) {
			cvs->strat->coord_goal[i][OPP_CAP_STATE] = empty;

			//if some oponent is actually for more than 1scan on the same spot reduce % of goal being there
		}
		else if (cvs->strat->coord_goal[i][OPP_CAP_STATE] == capturingCheckOk)
		{
			if (cvs->strat->avilability[i] > 0)
			{
				cvs->strat->avilability[i] -= 100 / capturingDetectionCoeff * (cvs->inputs->t - lastT);
			}
			//			std::cout << "goal %:" << cvs->strat->avilability[i] << "\n";
			cvs->strat->coord_goal[i][OPP_CAP_STATE] = capturingCheck;
		}
		// set up first capturingCheck after an oponent was registered on goal
		else if (cvs->strat->coord_goal[i][OPP_CAP_STATE] == capturing) {
			cvs->strat->coord_goal[i][OPP_CAP_STATE] = capturingCheck;
		}
	}
	lastT = cvs->inputs->t;

}

// Gives a score to every goal and determines the best one to go.
static int determine_goal(CtrlStruct *cvs)
{
	int i = 0;
	int index = 0;
	double distOpp2target = 0, dist2target = 0, score, max = -10;
	for (i = 0; i < NB_TARGETS; i++)
	{
		// Rajouter index dej pris
		distOpp2target = getDistanceFromAtoB(std::pair<int, int>(cvs->opp_pos->x[0] * 1000, cvs->opp_pos->y[0] * 1000), std::pair<int, int>(cvs->strat->coord_goal[i][0], cvs->strat->coord_goal[i][1])) / 1000;
		dist2target = getDistanceFromAtoB(std::pair<int, int>(cvs->rob_pos->x * 1000, cvs->rob_pos->y * 1000), std::pair<int, int>(cvs->strat->coord_goal[i][0], cvs->strat->coord_goal[i][1])) / 1000;
		score = (KN*cvs->strat->coord_goal[i][2] + distOpp2target - KD*dist2target)*cvs->strat->avilability[i] / 100;
		//		std::cout << "availability: " << cvs->strat->avilability[i] << " " << i << "\n";
		printf("%d:  %f \n\n", i, score);
		if (max == -10)
			max = score;
		if (score > max)
		{
			index = i;
			max = score;
		}
	}
	printf("INDEX NSM %d \n", index);
	return index;
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
	CtrlIn *inputs;
	static int captured = 0;

	// variables initialization
	inputs = cvs->inputs;

	//	double goal[2] = { 200,0 };
	//	std::pair<int, int> goal[2] = { std::pair<int,int>(-400, -400),
	//	std::pair<int,int>(400,800) };

	std::pair<int, int> momentaryGoal;
	momentaryGoal.first = cvs->strat->coord_goal[cvs->strat->index_goal][0];
	momentaryGoal.second = cvs->strat->coord_goal[cvs->strat->index_goal][1];

	static std::pair<int, int> noGoal(3666, 3666);
	std::pair<int, int> ngoal;
	std::pair<int, int> pos(cvs->rob_pos->x * 1000, cvs->rob_pos->y * 1000);
	static double timer = 0;
	static int goalSwitch = 0;

	if (cvs->inputs->t > RETURNTIME && cvs->strat->main_state != GAME_STATE_END && 
		cvs->strat->main_state != GAME_STATE_DRIVE_HOME) {
		ngoal = get_node_pos(BASE_NODE_NB);
		cvs->strat->main_state = GAME_STATE_DRIVE_HOME;
		set_goal(cvs->path, pos, ngoal);
	}


	evaluate_Oponent(cvs);
	//	determine_goal(cvs);
	std::cout << "gamestate: " << cvs->strat->main_state << "\n";
	switch (cvs->strat->main_state)
	{
	case GAME_STATE_START:

		if (cvs->inputs->t > 0)
		{
			init_djikstra(cvs);
			cvs->strat->main_state = GAME_STATE_SET_NEW_GOAL;
			timer = cvs->inputs->t;
		}
		path_planning(cvs);
		follow_path(cvs, cvs->path->theta, cvs->path->linspeed, cvs->rob_pos->theta);
		std::cout << "intermediate goal: " << cvs->path->nextGoal[0] << " " << cvs->path->nextGoal[1] << "\n";
		break;

	case GAME_STATE_DRIVE:
		std::cout << "goal: " << cvs->path->nextGoal[0] << " " << cvs->path->nextGoal[1] << "\n";
		if (get_Distance(momentaryGoal, pos) < 30) {
			std::cout << "_ strat arrived at target\n";
			set_goal(cvs->path, pos, noGoal);
			cvs->strat->main_state = GAME_STATE_Capture;					// wait 3sec
			captured = cvs->inputs->nb_targets;
			timer = cvs->inputs->t;
//			std::cout << "timer started: " << (cvs->inputs->t - timer) << "\n";
			break;
		}
		update_path_planning(cvs);
		path_planning(cvs);
		follow_path(cvs, cvs->path->theta, cvs->path->linspeed, cvs->rob_pos->theta);
		break;

	case GAME_STATE_Capture:
		if (1){//cvs->inputs->target_detected) {
//			std::cout << "capturing " << (cvs->inputs->t - timer) << " " << (cvs->inputs->nb_targets - captured) << "\n";
			if ((cvs->inputs->t - timer)>4 || cvs->inputs->nb_targets > captured) {		//cvs->strat->avilability[cvs->strat->index_goal] = 0;												//todo change to exact secs
				std::cout << "captured\n";
					cvs->strat->avilability[cvs->strat->index_goal] = 0;

				
				// set new goal
				cvs->strat->main_state = GAME_STATE_SET_NEW_GOAL; 
				std::cout << "old goal id: " << cvs->strat->index_goal << "availability: " << cvs->strat->avilability[cvs->strat->index_goal] << "\n";

				break;
			}
			else {
			}
		}
		else {	// we are not on target!													//todo change to exact secs
			std::cout << "not on tg\n";
			cvs->strat->main_state = GAME_STATE_SET_NEW_GOAL;

//			cvs->strat->avilability[cvs->strat->index_goal] = cvs->strat->avilability[cvs->strat->index_goal] / 1.02;
			//todo: what if target is gone?
		}
		//path_planning(cvs);
		speed_regulation(cvs, 0, 0);

		break;

	case GAME_STATE_SET_NEW_GOAL:

		std::cout << "new goal should be set" << (cvs->inputs->t - timer) << "\n";
		if (cvs->inputs->nb_targets <2) {
			cvs->strat->index_goal = determine_goal(cvs);
			ngoal = std::pair<int, int>(cvs->strat->coord_goal[cvs->strat->index_goal][0],
				cvs->strat->coord_goal[cvs->strat->index_goal][1]);
			cvs->strat->main_state = GAME_STATE_DRIVE;
		}
		else {

			std::cout << "setting base as target\n";
			cvs->strat->main_state = GAME_STATE_DRIVE_HOME;
			ngoal = get_node_pos(BASE_NODE_NB);
		}

		set_goal(cvs->path, pos, ngoal);
		std::cout << "_strat goal set: " << ngoal.first << " " << ngoal.second << "\n";

		std::cout << "_strat next goal on path: " << cvs->path->nextGoal[0] << " " << cvs->path->nextGoal[1] << " availability endgoal: " << cvs->strat->avilability[cvs->strat->index_goal] << "\n";
		break;

	case GAME_STATE_DRIVE_HOME:
		
		if (get_Distance(get_node_pos(BASE_NODE_NB), pos) < 30) {
			std::cout << "_ strat arrived home\n";
			set_goal(cvs->path, pos, noGoal);
			if (!cvs->outputs->flag_release)
				cvs->outputs->flag_release = 1;
			else {
				captured = 0;
				cvs->outputs->flag_release = 0;
				if (cvs->inputs->t < 80)
					cvs->strat->main_state = GAME_STATE_SET_NEW_GOAL;
				else
					cvs->strat->main_state = GAME_STATE_END;
			}
			timer = cvs->inputs->t;
			break;
		}
		std::cout << "driving home: " << cvs->path->nextGoal[0] << " " << cvs->path->nextGoal[1];
		update_path_planning(cvs);
		path_planning(cvs);
		follow_path(cvs, cvs->path->theta, cvs->path->linspeed, cvs->rob_pos->theta);
		break;

	case GAME_STATE_END:
	{
		speed_regulation(cvs, 0, 0);
	}

	default:
		printf("Error: unknown strategy main state: %d !\n", cvs->strat->main_state);
		exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
