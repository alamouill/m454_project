/*! 
 * \author Group 7
 * \file path_planning_gr7.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR7_H_
#define _PATH_PLANNING_GR7_H_ 

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <limits.h>
#include <vector>
#include "namespace_ctrl.h"
#include "CtrlStruct_gr7.h"

NAMESPACE_INIT(ctrlGr7);

/// path-planning main structure
struct PathPlanning
{
	std::vector<int> vectPath;
	double nextGoal[2];
	double speed[2];
	double theta;
	double linspeed;
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

void set_goal(PathPlanning *path, std::pair<int, int> pos, std::pair<int, int> goal);

void path_planning(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
