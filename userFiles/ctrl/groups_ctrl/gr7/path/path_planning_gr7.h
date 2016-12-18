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
//	std::vector<int> vectPath;
//	std::vector<int>* testPath;
	int startID;
	int goalID;
	int positionOnPath = 0;
	double nextGoal[2];
	double speed[2];
	double theta;
	double linspeed;
};

PathPlanning* init_path_planning(CtrlStruct *cvs);
void free_path_planning(PathPlanning *path);
void init_djikstra(CtrlStruct*cvs);

void set_goal(PathPlanning *path, std::pair<int, int> pos, std::pair<int, int> goal);
void update_path_planning(CtrlStruct *cvs);
void path_planning(CtrlStruct *cvs);
double getDistanceFromAtoB(std::pair<int, int> startPos, std::pair<int, int> goalPos);
std::pair<int, int> get_node_pos(int node_nb);

NAMESPACE_CLOSE();

#endif
