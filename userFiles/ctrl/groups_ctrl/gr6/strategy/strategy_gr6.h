/*! 
 * \author Group 6
 * \file strategy_gr6.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_GR6_H_
#define _STRATEGY_GR6_H_

#include "CtrlStruct_gr6.h"

NAMESPACE_INIT(ctrlGr6);

/// strategy main structure
struct Strategy
{
	int main_state; ///< main state of the strategy

	// bool to know if we are in a base zone or not
	bool at_base;

	// objectif to go to
	float objectif_x; 
	float objectif_y;
	bool going_base;

	// taking target parameters

	// time staying still to take target
	double t_still;
	// number of target carried before trying to take the new target
	int nb_target_before;
	// number of time we tried to take the new target
	int count;

	// wiggle parameters
	double wiggle_t;
	int wiggle_state;
};

/// 'main_state' states (adapt with your own states)
enum {GAME_STATE_DECIDE, GAME_STATE_FIND_PATH, GAME_STATE_FOLLOW_PATH, GAME_STATE_AVOID, GAME_STATE_LAY_TARGET, GAME_STATE_TAKE_TARGET, GAME_STATE_WIGGLE,GAME_STATE_FINISH};

// 
#define TAKE_TARGET_T_STILL 3 // time to stay still to take target
#define TAKE_TARGET_TRY_MAX 3 // number of time to try to take the target at maximum
#define WIGGLE_TIME 1 // time of wiggling
#define WIGGLE_SPEED 2 // speed of wiggling

// state for wiggling
enum{MOVE_BACK, TURN, MOVE_FRONT};

Strategy* init_strategy();
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);

void strat_decidePointToGo(CtrlStruct* cvs);
void strat_findPath(CtrlStruct*cvs);
void strat_followPath(CtrlStruct* cvs);
void strat_avoid(CtrlStruct* cvs);
void strat_lay_target(CtrlStruct* cvs);
void strat_take_target(CtrlStruct* cvs);
void strat_wiggle(CtrlStruct* cvs);
void strat_finish(CtrlStruct* cvs);

NAMESPACE_CLOSE();

#endif
