/*!
* \author Group 6
* \file path_regulation_gr6.h
* \brief regulation to follow a given path
*/

#ifndef _PATH_REGULATION_GR6_H_
#define _PATH_REGULATION_GR6_H_

#include "CtrlStruct_gr6.h"

NAMESPACE_INIT(ctrlGr6);

// circle data
enum { C_X, C_Y, C_R, C_NB_DATA };

// 2d point data
#define SPACE_DIMENSION 2


void follow_path(CtrlStruct *cvs);
void distance_to_obstacle(double point_x, double point_y, Obstacle *obs);
double dist_point_to_edge(double point_x, double point_y, double edge_start[SPACE_DIMENSION], double edge_end[SPACE_DIMENSION], double length, double* prox_x, double* prox_y);
bool inter_edge_circle(double circle[C_NB_DATA], double edge_start[SPACE_DIMENSION], double edge_end[SPACE_DIMENSION], double length);

int follow_path(CtrlStruct *cvs, int *nodeNumber);
void move_robot(CtrlStruct *cvs, float x, float y);

void move_to_node(CtrlStruct *cvs);
void path_nextNode(CtrlStruct* cvs);
void check_at_node(CtrlStruct* cvs);

NAMESPACE_CLOSE();

#endif
