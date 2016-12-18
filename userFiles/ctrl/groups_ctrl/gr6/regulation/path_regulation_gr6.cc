#include "path_regulation_gr6.h"
#include "init_pos_gr6.h"
#include "useful_gr6.h"
#include "speed_regulation_gr6.h"
#include "opp_pos_gr6.h"
#include "path_planning_gr6.h"
#include "kalman_gr6.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr6);
#define DISTANCE_MAX 4.0

enum { JULIEN, LUCIE }; // driving method

/*! \brief follow a given path
*
* \param[in,out] cvs controller main structure
*/
void follow_path(CtrlStruct *cvs)
{
	return;
}

// compute the minimum distance from a point_(x,y) to all the obstacles in obs
void distance_to_obstacle(double point_x, double point_y, Obstacle *obs)
{
	// Variable declaration
	int i = 0, nb_corner;
	double edge_start[SPACE_DIMENSION], edge_end[SPACE_DIMENSION];
	double length;
	double min_distance, dist;
	double proxima_x, proxima_y;
	double prox_x, prox_y; // nearest point of the segment from the robot

	// Variable initiation
	nb_corner = obs->nb_corner;
	min_distance = DISTANCE_MAX; // as it will be compared  to choose the minima, it has to be overestimated (here we chose 4 m as the maximum distance is 3.6m)

	// Compute the orthogonal projection
	for (i = 0; i < nb_corner; i++)
	{
		//initialisation
		edge_start[P_X] = obs->corner_posx[i];
		edge_start[P_Y] = obs->corner_posy[i];
		length = obs->length[i];

		// when we reach the end of the table we take the first point for the end and the last for the start of the last segment
		if (i == nb_corner - 1){ 
			edge_end[P_X] = obs->corner_posx[0];
			edge_end[P_Y] = obs->corner_posy[0];
		}
		else{
			edge_end[P_X] = obs->corner_posx[i + 1];
			edge_end[P_Y] = obs->corner_posy[i + 1];
		}

		// calcul of distance from point to edge
		dist = dist_point_to_edge(point_x,point_y,edge_start, edge_end, length, &prox_x,&prox_y);
		
		// update minimal distance
		if (min_distance > dist) {
			min_distance = dist;
			proxima_x = prox_x;
			proxima_y = prox_y;
		}
	}

	//update of the obstacle structure
	obs->distance = min_distance;
	obs->proxima_x = proxima_x;
	obs->proxima_y = proxima_y;

	return;
}

// orientation PI controler
double orientation_regulation(CtrlStruct *cvs, float angle) {
	float K = 50;
	float Ki = 0.1;
	float a_commande;
	double dt, int_error_angle;
	PathPlanning* path;
	CtrlIn* inputs;

	// initialisation
	path = cvs->path;
	inputs = cvs->inputs;
	dt = inputs->t - path->last_ta;

	//Integral term of the error (ARW)
	int_error_angle = (path->int_error_angle) + angle*dt;

	//Limitation of Integral term of the error (ARW)
	int_error_angle = limit_range(int_error_angle, -50, 50);

	//PI_regulation
	a_commande = angle*K + Ki*int_error_angle;

	// time update
	path->last_ta = inputs->t;

	// update Integral term
	path->int_error_angle = int_error_angle;

	return a_commande;
}

// move to the next node given by path->nextNode
void move_to_node(CtrlStruct *cvs) {

	PathPlanning *path = cvs->path;
	float rot_angle = 0, theta_rob = 0;
	float delta_trajet = 0, dx = 0, dy = 0;
	float kal_x = cvs->kalman->kalman_pos.x(), kal_y = cvs->kalman->kalman_pos.y();
	float kal_theta = cvs->kalman->kalman_pos.z();

	//objectif
	float target_x = path->nextNode->posx, target_y = path->nextNode->posy;

	// distance to target in meter
	dx = target_x - kal_x;
	dy = target_y - kal_y;
	delta_trajet = sqrt(dx*dx + dy*dy);

	// rotation angle that the robot needs to do (in rad)
	rot_angle = (DEG_TO_RAD*kal_theta - atan2(dy, dx));

	// replace the rotation angle between -M_PI and M_PI rad 
	if (rot_angle >= M_PI)
		rot_angle = rot_angle - 2 * M_PI;
	if (rot_angle < -M_PI)
		rot_angle = rot_angle + 2 * M_PI;

	// change mode of path regulation
	int mode = JULIEN;

	if (mode == LUCIE) {
		// Send the command to the motors
		// if at target position, dont move
		if ((delta_trajet < TRAJET_MIN) && (fabs(rot_angle) < ANGLE_PRECISION)) // stop the movement if we are at the goal with a certain tolerance
		{
			speed_regulation(cvs, 0, 0);
		}
		// if not at target position, move
		else {
			double a_command = orientation_regulation(cvs, rot_angle);
			speed_regulation(cvs, 10 - a_command, 10 + a_command);
			//set_plot(a_command, "ang command");
		}
	}
	else if (mode == JULIEN) {
		if (delta_trajet > TRAJET_MIN) {
			if (fabs(rot_angle) > ANGLE_PRECISION) {
				if (rot_angle < 0) {
					speed_regulation(cvs, 10, -10);
				}
				else { //if (rot_angle > 0)
					speed_regulation(cvs, -10, 10);
				}
			}
			else { //if (fabs(rot_angle) < ANGLE_PRECISION) {
				speed_regulation(cvs, 15, 15);
			}
		}
		else {
			speed_regulation(cvs, 0, 0);
		}
	}
	else {
		return;
	}

	return;
}

// update nextNode to go to
void path_nextNode(CtrlStruct* cvs) {
	PathPlanning* path = cvs->path;

	// get the number of node that are in the optimal path
	int nb_node = 0;
	for (nb_node = 0; nb_node < NB_NODE; nb_node++) {
		if (path->nodeNumber[nb_node] == -1) {
			break;
		}
	}

	// increase index of node to go to
	(path->next_node_index)++;

	// if node_index == nb_node, it's the final node 
	if (path->next_node_index == nb_node) {
		path->next_node_index = 0;
		path->path_finalNode = true;
		path->path_baseNode = path->nextNode->base;
		return;
	}
	
	// if not the last one, update nextNode
	path->nextNode = path->nodeslist[path->nodeNumber[path->next_node_index]];
	path->path_baseNode = path->nextNode->base;
	return;
}

// check if at node while following the pathh
void check_at_node(CtrlStruct* cvs) {
	float delta_trajet = 0, dx = 0, dy = 0;

	// calculate the distance to the node
	dx = cvs->path->nextNode->posx - cvs->kalman->kalman_pos.x();
	dy = cvs->path->nextNode->posy - cvs->kalman->kalman_pos.y();
	delta_trajet = sqrt(dx*dx + dy*dy);

	// if close enough to node
	if (delta_trajet < TRAJET_MIN) {
		cvs->path->arrivedAtNode = true;
	}
	else {
		cvs->path->arrivedAtNode = false;
	}
	return;
}

// compute to see if an edge and a circle intersect, if intersect, return true, else return false
bool inter_edge_circle(double circle[C_NB_DATA], double edge_start[SPACE_DIMENSION], double edge_end[SPACE_DIMENSION], double length) {
	double dist = 0, prox_x = 0, prox_y = 0;

	// calcul of distance from center of circle to edge
	dist = dist_point_to_edge(circle[C_X], circle[C_Y], edge_start, edge_end, length, &prox_x, &prox_y);

	if (dist <= circle[C_R]) {
		return true;
	}
	else {
		return false;
	}
}

// calculate the distance from point(x,y) to edge, and modify prox_(x,y) to be the closest point of edge to the point
double dist_point_to_edge(double point_x, double point_y, double edge_start[SPACE_DIMENSION], double edge_end[SPACE_DIMENSION], double length, double* prox_x, double* prox_y) {
	
	double orientation_x = 0, orientation_y = 0;
	double vector_x, vector_y; // vector formed by the point and the start of the straight line
	double test=0; // parameter which test if the orthogonal projection is inside (0<test<1) the segment, before the start (test<0) or after the end (test>1)
		
	// calculation of the orientation vector of the straigh line
	orientation_x = edge_end[P_X] - edge_start[P_X];
	orientation_y = edge_end[P_Y] - edge_start[P_Y];

	// calculation of the vector robot-start of the segment
	vector_x = point_x - edge_start[P_X];
	vector_y = point_y - edge_start[P_Y];

	// test of the position of the projection
	test = (vector_x*orientation_x + vector_y*orientation_y) / pow(length, 2);
	if (test < 0) {
		*prox_x = edge_start[P_X];
		*prox_y = edge_start[P_Y];
	}
	else if (test > 1) {
		*prox_x = edge_end[P_X];
		*prox_y = edge_end[P_Y];
	}
	else {
		*prox_x = edge_start[P_X] + test*(edge_end[P_X] - edge_start[P_X]);
		*prox_y = edge_start[P_Y] + test*(edge_end[P_Y] - edge_start[P_Y]);
	}

	//calculation of the distance
	return sqrt(pow((*prox_x - point_x), 2) + pow((*prox_y - point_y), 2));
}

// not used
// follow a path given by a node list index
int follow_path(CtrlStruct *cvs, int *nodeNumber) {
	PathPlanning* path = cvs->path;
	path_nextNode(cvs);
	move_to_node(cvs);
	set_plot(path->next_node_index, "node index");
	return 0;
}

// not used but maybe usefull
// do the final path from node to x,y
int final_path(CtrlStruct *cvs, float x, float y) {
	move_to_node(cvs);

	// calculate the distance to final point
	float dx = 0, dy = 0, delta_trajet = 0;
	dx = x - cvs->rob_pos->x;
	dy = y - cvs->rob_pos->y;
	delta_trajet = sqrt(dx*dx + dy*dy);
	if (delta_trajet < TRAJET_MIN) {
		return 1;
	}
	return 0;
}

// not used
// move to given final position
void move_robot(CtrlStruct *cvs, float x, float y) {
	PathPlanning* path = cvs->path;
	RobotPosition* rob_pos = cvs->rob_pos;
	double dist = 0; // not used but need to be send to find opt path

	if (!path->path_created) {
		path->nodeNumber = find_opt_path(x, y, rob_pos->x, rob_pos->y, cvs, &dist);
		path->path_created = true;
	}

	if (path->path_created) {
		if (!path->arrivedAtNode) {
			if (!path->path_finalNode) {
				if (follow_path(cvs, path->nodeNumber)) {
					path->path_finalNode = true;
				}
			}
			if (path->path_finalNode) {
				if (final_path(cvs, x, y)) {
					path->arrivedAtNode = true;
					printf("arrivedAtNode\n");
				}
			}
		}
		if (path->arrivedAtNode) {
			speed_regulation(cvs, 0, 0);
		}
	}

	return;
}

// not used
// find next node
int path_findnextNode(CtrlStruct* cvs) {
	float delta_trajet = 0, dx = 0, dy = 0;
	RobotPosition *rob_pos = cvs->rob_pos;
	PathPlanning* path = cvs->path;
	int *nodeNumber = path->nodeNumber;

	int nb_node = 0;
	for (nb_node = 0; nb_node < NB_NODE; nb_node++) {
		if (nodeNumber[nb_node] == -1) {
			break;
		}
	}

	// get the node to go to based on the index
	path->nextNode = path->nodeslist[nodeNumber[path->next_node_index]];

	// calculate the distance to the node
	dx = path->nextNode->posx - cvs->rob_pos->x;
	dy = path->nextNode->posy - cvs->rob_pos->y;
	delta_trajet = sqrt(dx*dx + dy*dy);
	if (delta_trajet < TRAJET_MIN) {
		(path->next_node_index)++;
		if (path->next_node_index == nb_node) {
			path->next_node_index = 0;
			return 1;
		}
	}
}

NAMESPACE_CLOSE();
