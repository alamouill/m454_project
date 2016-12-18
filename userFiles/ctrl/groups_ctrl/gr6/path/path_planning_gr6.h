/*! 
 * \author Group 6
 * \file path_planning_gr6.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR6_H_
#define _PATH_PLANNING_GR6_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr6.h"

NAMESPACE_INIT(ctrlGr6);

#define NB_NODE 18	// number of nodes on plan
#define MAX_VOISIN 11 // maximum number of node's neighbours
#define MAX_WALL 12

#define NB_OBSTACLE 6 //lucie
#define NB_CORNER 32
#define MAX_CORNER 12 // All the elements of the structure have to be the same size
#define P_X 0
#define P_Y 1

#define DEG_TO_RAD (M_PI/180.0) //< conversion from degrees to radians
#define RAD_TO_DEG (1/DEG_TO_RAD) //< conversion from radians to degrees

#define ANGLE_PRECISION 1*DEG_TO_RAD //< prec
#define TRAJET_MIN 0.020 //< position precision [m]

// Obstacle structure
struct Obstacle {
	// number and characteristics
	int number;
	int nb_corner;

	// angle position
	double corner_posx[MAX_CORNER];
	double corner_posy[MAX_CORNER];

	// segment length
	double length[MAX_CORNER];

	// coordonate of the nearest point between the obstacle and the robot and distance
	double distance;
	double proxima_x;
	double proxima_y;

};

// path-planning main structure
struct PathPlanning
{
	Node* nodeslist[NB_NODE];	// node list which lists all of the nodes that can be moved to
	Obstacle* obstacle_list[NB_OBSTACLE];		// obstacle list which lists all of the fix obstacle that should be avoided

	// parameters related to the path to follow
	bool path_created;		// bool to say if path has been calculated ( true if created, false if not )
	bool path_finalNode;	// bool to say if path has been followed till the end ( true if followed, false if not )
	bool arrivedAtNode;		// bool to say if the robot is arrived at node
	bool path_baseNode;		// bool to say if node is a base node

	int* nodeNumber;		// node number to follow when following the path
	int next_node_index;	// to see at what node we are supposed to go ( based on nodeNumber )
	Node* nextNode;			// next node to go to

	// parameter for driving
	double last_t;
	double last_ta;
	double int_error_angle;

	// Newton force for obstacle avoidance
	double f_att_x;
	double f_att_y;
	double f_rep_x;
	double f_rep_y;

	// Additional speed command from the potential field
	double command_v;
	double command_w;

};

// node structure
struct Node {
	// position
	double posx;
	double posy;

	// neighbour
	int numero; // own number
	int nb_voisin; // number of direct neighbours
	int liste_voisin[MAX_VOISIN]; // list of index of direct neighbours
	double weight_edge[MAX_VOISIN]; // weight of the edge with the neighbour at the same index of list_ngb
	bool accessible[MAX_VOISIN]; // true if edge is accesible at the same index of list_ngb

	// poids 
	int  weight; // point of the target on the node
	bool base; // true if node is a base node
	bool inter; // true if node is a intermediary node

	// bool to know if target is taken
	bool taken; // 1 if taken, 0 if not taken ( 1 for all node without target )

};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);
int* find_opt_path(double x, double y, double posx, double posy, CtrlStruct* cvs, double *dist);
void path_planning_correct_init(CtrlStruct *cvs);
void path_decision_making(CtrlStruct* cvs);
void path_update_taken(CtrlStruct* cvs);
bool can_reach(double x, double y, double px, double py, PathPlanning* path);

NAMESPACE_CLOSE();

#endif