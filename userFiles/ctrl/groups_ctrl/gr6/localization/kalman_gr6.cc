#include "kalman_gr6.h"
#include "odometry_gr6.h"
#include "triangulation_gr6.h"
#include "useful_gr6.h"
#include <iostream>
#include <Eigen/Dense>

using Eigen::Matrix3d;
using Eigen::Vector3d;

NAMESPACE_INIT(ctrlGr6);

#define R_COEF 0.00000000001
#define Q_COEF 0.00000001 //augmenter pour réduire l'influence de la triangulation

/*! \brief follow a given path
*
* \param[in,out] cvs controller main structure
*/
void kalman(CtrlStruct *cvs)
{

	// variable declaration
	RobotPosition *rob_pos;
	RobotPosition *triang_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	triang_pos = cvs->triang_pos;

	//set_plot(triang_pos->x, "tri_x");
	//set_plot(triang_pos->y, "tri_y");
	set_plot(triang_pos->theta, "tri_theta");

	//constant declaration
	Matrix3d R;
	R << R_COEF, 0, 0, 0, R_COEF, 0, 0, 0, R_COEF;
	Matrix3d Q;
	Q << Q_COEF, 0, 0, 0, Q_COEF, 0, 0, 0, Q_COEF;
	Matrix3d J;
	//	J << 1, 0, 0, 0, 1, 0, sin(rob_pos->theta*M_PI/180), cos(rob_pos->theta*M_PI/180), 1;
	J << 1, 0, 0, 0, 1, 0, -cvs->kalman->ds*sin(rob_pos->theta), cvs->kalman->ds* cos(rob_pos->theta), 1;
	//J << 1, 0, 0, 0, 1, 0,0, 0, 1;

	//vector declaration
	Vector3d rob_pos_vect;
	rob_pos_vect << rob_pos->x, rob_pos->y, rob_pos->theta;

	Vector3d triang_pos_vect;
	triang_pos_vect << triang_pos->x, triang_pos->y, triang_pos->theta;

	Vector3d delta = rob_pos_vect - cvs->kalman->prev_odo;

	//PREDICTION STEP
	Vector3d x_pred;
	x_pred = cvs->kalman->kalman_pos + delta;

	//matrix of the variance of the estimation
	Matrix3d sigma_predicted = J*cvs->kalman->prev_sigma*J.transpose() + R;

	//Kalman's gain
	Matrix3d H;
	H << 1, 0, 0, 0, 1, 0, 0, 0, 1; //useless but keep to be similar to the initial algo

	Matrix3d temp = H*sigma_predicted*H.transpose() + Q;
	temp = temp.reverse();
	Matrix3d K = sigma_predicted*H.transpose() * temp;

	//MEASURE STEP
	cvs->kalman->kalman_pos = x_pred + K*(triang_pos_vect - H*x_pred);

	//covariance matrix
	Matrix3d I;
	I << 1, 0, 0, 0, 1, 0, 0, 0, 1;

	Matrix3d sigma = (I - K*H)*sigma_predicted;

	//updating
	cvs->kalman->prev_odo = rob_pos_vect;
	cvs->kalman->prev_tri = triang_pos_vect;
	cvs->kalman->prev_sigma = sigma;

	//plot to test Kalman
	//set_plot(cvs->kalman->kalman_pos.x(), "kal_x");
	//set_plot(cvs->kalman->kalman_pos.y(), "kal_y");
	//set_plot(cvs->kalman->kalman_pos.z()*M_PI / 180, "kal_theta");

	// for now, until kalman works
	// cvs->kalman->kalman_pos = rob_pos_vect;
	
	return;
}

KalmanStruct* init_kalman()
{
	KalmanStruct* kalman = (KalmanStruct*)malloc(sizeof(KalmanStruct));
	kalman->kalman_pos << 0, 0, 0;
	kalman->prev_odo << 0, 0, 0;
	kalman->prev_tri << 0, 0, 0;
	kalman->prev_sigma << 0, 0, 0, 0, 0, 0, 0, 0, 0;

	return kalman;
}

void correct_init_kalman(CtrlStruct* cvs) {
	
	RobotPosition* rob_pos = cvs->rob_pos;
	
	cvs->kalman->kalman_pos << rob_pos->x, rob_pos->y, rob_pos->theta;
	cvs->kalman->prev_odo << rob_pos->x, rob_pos->y, rob_pos->theta;
	cvs->kalman->prev_tri << rob_pos->x, rob_pos->y, rob_pos->theta;

	return;
}

NAMESPACE_CLOSE();
