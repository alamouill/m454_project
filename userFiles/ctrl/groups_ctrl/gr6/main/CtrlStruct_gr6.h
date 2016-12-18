/*! 
 * \author Group 6
 * \file CtrlStruct_gr6.h
 * \brief Controller main structure
 */

#ifndef _CTRL_STRUCT_GR6_H_
#define _CTRL_STRUCT_GR6_H_

#include "ctrl_io.h"
#include "namespace_ctrl.h"
#include "user_realtime.h"
#include "set_output.h"
#include <stdlib.h>
#include <stdio.h>

NAMESPACE_INIT(ctrlGr6);

/// main states
enum {CALIB_STATE, WAIT_INIT_STATE, RUN_STATE, STOP_END_STATE, NB_MAIN_STATES};

/// robot IDs
enum {ROBOT_B, ROBOT_R, ROBOT_Y, ROBOT_W, NB_ROBOTS};

/// teams
enum {TEAM_A, TEAM_B, NB_TEAMS};

#define DEG_TO_RAD (M_PI/180.0) //< conversion from degrees to radians
#define RAD_TO_DEG (1/DEG_TO_RAD) //< conversion from radians to degrees
#define TOWER_RADIUS (40*0.001) // radius of the tower 40mm
#define DIST_ROB_TOW (83*0.001) // distance from center of robot to center of tower 83mm
#define R_OPP 0.182
#define R_PROTECTION_CIRCLE (150*0.001) //protection circle for opponent detection m

// forward declaration
typedef struct RobotPosition RobotPosition;
typedef struct SpeedRegulation SpeedRegulation;
typedef struct RobotCalibration RobotCalibration;
typedef struct OpponentsPosition OpponentsPosition;
typedef struct PathPlanning PathPlanning;
typedef struct Strategy Strategy;
typedef struct Node Node;
typedef struct Obstacle Obstacle;
typedef struct KalmanStruct KalmanStruct;

/// Main controller structure
typedef struct CtrlStruct
{
	CtrlIn *inputs;   ///< controller inputs
	CtrlOut *outputs; ///< controller outputs

	RobotPosition *rob_pos;     ///< robot position coming from the robot odometry
	RobotPosition *triang_pos;  ///< robot position coming from the triangulation
	OpponentsPosition *opp_pos; ///< opponents position
	SpeedRegulation *sp_reg;    ///< speed regulation
	RobotCalibration *calib;    ///< calibration
	PathPlanning *path;         ///< path-planning
	Strategy *strat;            ///< strategy
	KalmanStruct *kalman;		///< kalman filter

	int main_state; ///< main state
	int robot_id;   ///< ID of the robot
	int team_id;    ///< ID of the team
	int nb_opp;     ///< number of opponents

} CtrlStruct;

// function prototypes
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs);
void free_CtrlStruct(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
