/* Mobile Robots Project - kalman_gr7.cc
 * Kalman Filter on robot position using odometry and triangulation
 * Version 2.1
 * Last Update: 18/12/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 

#include "kalman_gr7.h"
#include "odometry_gr7.h"
#include "triangulation_gr7.h"
#include "useful_gr7.h"
#include <iostream>

#define R_ROUE 0.03
#define ECART_ROUE 0.225
#define QDIAG 0.11
#define QTHETA 0.016
#define RCOEFF 1.518

NAMESPACE_INIT(ctrlGr7);

void inverse_matrix_33(double m[3][3], double minv[3][3])
{
	double det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
		m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
		m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
	double invdet = 1 / det;

	//printf("Determinant: %f, inverse det: %f", det, invdet);

	minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
	minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
	minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
	minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
	minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
	minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
	minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
	minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
	minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;

}

void mult_matrices(double a[3][3], double b[3][3], double result[3][3])
{
	int i = 0, j = 0, k = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			for (k = 0; k < 3; k++)
			{
				result[i][j] += a[i][k] * b[k][j];
			}

		}
	}
}

void transpose_matrices(double matrice[3][3], double result[3][3])
{
	int i = 0, j = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			result[j][i] = matrice[i][j];
		}
	}
}

void sum_matrices(double a[3][3], double b[3][3], double result[3][3])
{
	int i = 0, j = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			result[i][j] = a[i][j] + b[i][j];
		}
	}
}


void substract_matrices(double a[3][3], double b[3][3], double result[3][3])
{
	int i = 0, j = 0;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			result[i][j] = a[i][j] - b[i][j];
		}
	}
}

// new_position est un tableau dans lequel sera enregitré la nouvelle position (x,y,theta).
// measured_position est un tableau contenant la mesure faite par triangulation (x,y,theta).
// command est un tableau contenant la commande de vitesse envoyée à chaque roue (w_r, w_l)
// dt est l'interval de temps considéré.
// theta est l'orientation du robot
void kalman(double new_position[3], double odometry_position[3], double tria_position[3], float command[2], float dt)
{
	int i = 0, j = 0;
	/*------------------------ Déclaration des matrices ---------------------*/
	static int init = false; 
	static double prev_position[3] = { 0,0,0 };
	double future_position[3] = { 0,0,0 };
	double F[3][3] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };
	float coefB = R_ROUE / 2 * dt;
	double B[3][2] = { { coefB*cos(odometry_position[2]),coefB*cos(odometry_position[2]) },
	{ coefB*sin(odometry_position[2]),coefB*sin(odometry_position[2]) },
	{ coefB * 1 / ECART_ROUE,-coefB * 1 / ECART_ROUE } };

	static double prev_P[3][3] = { {1,0,0},{ 0,1,0 },{ 0,0,1 } };

	double new_P[3][3] = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };
	double future_P[3][3] = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };
	double Q[3][3] = { { QDIAG,0,QTHETA },{ 0,QDIAG,QTHETA },{ QTHETA,QTHETA,QDIAG } };    ////////// COEF A DETERMINER!!
	double innovation[3] = { 0,0,0 };
	double R[3][3] = { { RCOEFF,0,0 },{ 0,RCOEFF,0 },{ 0,0,RCOEFF } };    ////////// COEF A DETERMINER!!
	double S[3][3] = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };
	double inv_S[3][3] = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };
	double K[3][3] = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };
	double KHP[3][3] = { { 0,0,0 },{ 0,0,0 },{ 0,0,0 } };

	/*-------------------------Firts Step: predict --------------------------*/

	if (!init)
	{
		prev_position[0] = odometry_position[0];
		prev_position[1] = odometry_position[1];
		prev_position[2] = odometry_position[2];
		init = true;
		printf("initialized kalman\n");
	}
	//new_position = F*prev_position+B*Command

//	new_position[0] = prev_position[0] + B[0][0] * command[0] + B[0][1] * command[1];
//	new_position[1] = prev_position[1] + B[1][0] * command[0] + B[1][1] * command[1];
//	new_position[2] = prev_position[2] + B[2][0] * command[0] + B[2][1] * command[1];

	new_position[0] = odometry_position[0];
	new_position[1] = odometry_position[1];
	new_position[2] = odometry_position[2];

//	new_position[2]= limit_angle(new_position[2]);

//	prev_position[0] = new_position[0];
//	prev_position[1] = new_position[1];
//	prev_position[2] = new_position[2];
	
	//new_P= F*prev_P*F^T+Q
	
	sum_matrices(prev_P, Q, new_P);

	/*-------------------------- Second Step: Update ------------------------*/

	//innovation= measure position-H*new_positiom

	innovation[0] = tria_position[0] - odometry_position[0];
	innovation[1] = tria_position[1] - odometry_position[1];
	innovation[2] = tria_position[2] - odometry_position[2];




	set_plot(tria_position[0], "tria X");
//	set_plot(tria_position[1], "tria Y");
//	set_plot(tria_position[2], "tria T");
	

	set_plot(odometry_position[0], "o X");
//	set_plot(odometry_position[1], "o Y");
//	set_plot(odometry_position[2], "o T");

	// S=H*new_P*H^T+R

	sum_matrices(new_P, R, S);
	
	//K=new_P*H^T*S^-1

	inverse_matrix_33(S, inv_S);
	mult_matrices(new_P, inv_S, K);

	//future_position=new_position + K*innovation

	future_position[0] = new_position[0] + K[0][0] * innovation[0] + K[0][1] * innovation[1] + K[0][2] * innovation[2];
	future_position[1] = new_position[1] + K[1][0] * innovation[0] + K[1][1] * innovation[1] + K[1][2] * innovation[2];
	future_position[2] = new_position[2] + K[2][0] * innovation[0] + K[2][1] * innovation[1] + K[2][2] * innovation[2];

	
	// future_P= (I-K*H)P

	mult_matrices(K, new_P, KHP);
	substract_matrices(new_P, KHP, future_P);

	/*************************************** Update new position *********************************/
 //   new_position[0]= first_order_filter(prev_position[0], future_position[0], 0.1, dt);
	//new_position[1]= first_order_filter(prev_position[1], future_position[1], 0.1, dt);
	//new_position[2]= first_order_filter(prev_position[2], future_position[2], 0.1, dt);
	new_position[0] = future_position[0];
	new_position[1] = future_position[1];
	new_position[2] = future_position[2];



	/*--------------------------------- Update for next call ---------------------------------*/

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			prev_P[i][j] = future_P[i][j];
		}
		prev_position[i] = future_position[i];
	}


//	std::cout << "tria: " << tria_position[0] << " odo: " << odometry_position[0] << " kal: " << new_position[0] << "\n";

}



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

	double position_post_kalman[3];

	double odo_pos[3] = { rob_pos->x,rob_pos->y,rob_pos->theta };
	
	
	
	double tria_pos[3] = { cvs->triang_pos->x, cvs->triang_pos->y, cvs->triang_pos->theta };
	// time
	float dt = cvs->rob_pos->last_dT;

	float command[2] = { cvs->outputs->wheel_commands[0] , cvs->outputs->wheel_commands[1] };

	
	kalman(position_post_kalman, odo_pos,tria_pos, command, dt);

	/*rob_pos->x = position_post_kalman[0];
	rob_pos->y = position_post_kalman[1];
	rob_pos->theta = position_post_kalman[2];*/
//	printf("%f \n", position_post_kalman[0]);
	set_plot(position_post_kalman[0], "our X[m]");
//	set_plot(position_post_kalman[1], "our Y[m]");
//	set_plot(position_post_kalman[2], "our Th[m]");
}

NAMESPACE_CLOSE();
