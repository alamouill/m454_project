/*! 
 * \author Group 7
 * \file path_planning_gr7.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR7_H_
#define _PATH_PLANNING_GR7_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr7.h"

NAMESPACE_INIT(ctrlGr7);

/// path-planning main structure
struct PathPlanning
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
