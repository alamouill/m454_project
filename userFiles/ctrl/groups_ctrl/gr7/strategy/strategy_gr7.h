/*!
* \author Group 7
* \file strategy_gr7.h
* \brief strategy during the game
*/

#ifndef _STRATEGY_GR7_H_
#define _STRATEGY_GR7_H_

#include "CtrlStruct_gr7.h"

NAMESPACE_INIT(ctrlGr7);
#define BASE_NODE_NB 8

/// strategy main structure
typedef struct Strategy
{
	int main_state; ///< main state of the strategy
	int coord_goal[8][4]; ///< x, y, score of each goal, oponent capturing state
	double avilability[8];
	int index_goal;
} Strategy;
//strategy struct enum
enum { X, Y, SCORE, OPP_CAP_STATE };

/// 'main_state' states (adapt with your own states)
enum { GAME_STATE_START, GAME_STATE_DRIVE, GAME_STATE_Capture, GAME_STATE_SET_NEW_GOAL, GAME_STATE_DRIVE_HOME, GAME_STATE_END };

/// oponents on target states
enum { empty, capturing, capturingCheck, capturingCheckOk, gone };

Strategy* init_strategy();
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
