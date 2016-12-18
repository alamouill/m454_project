/*!
* \author Group 6
* \file kalman_gr6.h
* \brief localization sensors fusion with Kalman
*/

#ifndef _KALMAN_GR6_H_
#define _KALMAN_GR6_H_

#include "CtrlStruct_gr6.h"
#include "init_pos_gr6.h"
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;


NAMESPACE_INIT(ctrlGr6);

/// Kalman main structure
typedef struct KalmanStruct
{
	// to store previous variables
	Vector3d prev_tri;
	Vector3d prev_odo;
	Matrix3d prev_sigma;
	//to store position calculated by kalman
	Vector3d kalman_pos;
	double ds;


}KalmanStruct;

KalmanStruct* init_kalman();
void correct_init_kalman(CtrlStruct* cvs);
void kalman(CtrlStruct *cvs);


NAMESPACE_CLOSE();

#endif
