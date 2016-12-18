#include "path_planning_gr6.h"
#include "init_pos_gr6.h"
#include "opp_pos_gr6.h"
#include "useful_gr6.h"
#include "speed_regulation_gr6.h"
#include <math.h>
#include "strategy_gr6.h"
#include "path_regulation_gr6.h"
#include "kalman_gr6.h"

NAMESPACE_INIT(ctrlGr6);

static void objectif_closest_base(CtrlStruct* cvs);
static int objectif_closest_target(CtrlStruct* cvs);
static void check_at_base(CtrlStruct* cvs);
static void objectif_target_base(CtrlStruct* cvs);
static void path_update_accessible_node(CtrlStruct* cvs);
static void path_reset_accessible_node(CtrlStruct* cvs);

#define TOL 0.00001

/*
node position in mm

1 point nodes
int node_x_1[4] = { 700, 700, -400, -400 }; 
int node_y_1[4] = { 600, -600, -600, 600 };

2 points nodes
int node_x_2[3] = { 250,   250,  100 };		
int node_y_2[3] = { -1250, 1250,   0 };

3 points nodes
int node_x_3 = -800;						
int node_y_3 = 0;

base nodes
int x_base[2] = { 750,   -700 };				
int y_base[2] = { 1200, -1200 };

nodes for driving around
int x_inter[8] = { -250,  400,  400, 400, 400,  350, 600, 300 };
int y_inter[8] = { -900, -500, -200, 200, 500, -800, 800, 800 };
*/
#define IND_PT_1 4
#define IND_PT_2 IND_PT_1 + 3
#define IND_PT_3 IND_PT_2 + 1
#define IND_BASE IND_PT_3 + 2
#define IND_DRIVING IND_BASE + 8
#define TARGET_BASE_NODE_INDEX 9

// node in mm
//							  |-----------1 pt node|------2 pts nodes|3 pts|---base node|--------------------------------driving node|
static int node_x[NB_NODE] = { 700, 700, -400, -400,   250,  250, 100, -800,  750,  -700,  -275,  400,  400, 400, 400,  350,  600, 300 };
static int node_y[NB_NODE] = { 600,-600, -600,  600, -1250, 1250,   0,    0, 1200, -1200, -1000, -550, -200, 200, 550, -800,  800, 800 };

// neighbour list
static int n0[MAX_VOISIN] = { 1, 3, 8,11,12,13,14,16,17,-1,-1 };	int nbv0 = 9;
static int n1[MAX_VOISIN] = { 0, 2, 8, 9,10,11,12,13,14,15,-1 };	int nbv1 = 10;
static int n2[MAX_VOISIN] = { 1, 4, 7,10,11,15,-1,-1,-1,-1,-1 };	int nbv2 = 6;
static int n3[MAX_VOISIN] = { 0, 5, 7,14,16,17,-1,-1,-1,-1,-1 };	int nbv3 = 6;
static int n4[MAX_VOISIN] = { 2, 9,10,15,-1,-1,-1,-1,-1,-1,-1 };	int nbv4 = 4;
static int n5[MAX_VOISIN] = { 3,17,-1,-1,-1,-1,-1,-1,-1,-1,-1 };	int nbv5 = 2;
static int n6[MAX_VOISIN] = { 12,13,-1,-1,-1,-1,-1,-1,-1,-1,-1 };	int nbv6 = 2;
static int n7[MAX_VOISIN] = { 2, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1 };	int nbv7 = 2;
static int n8[MAX_VOISIN] = { 0, 1,13,16,-1,-1,-1,-1,-1,-1,-1 };	int nbv8 = 4;
static int n9[MAX_VOISIN] = { 1, 4,10,11,-1,-1,-1,-1,-1,-1,-1 };	int nbv9 = 4;
static int n10[MAX_VOISIN] = { 1, 2, 4, 9,11,15,-1,-1,-1,-1,-1 };	int nbv10 = 6;
static int n11[MAX_VOISIN] = { 0, 1, 2, 9,10,12,15,-1,-1,-1,-1 };	int nbv11 = 7;
static int n12[MAX_VOISIN] = { 0, 1, 6,11,13,-1,-1,-1,-1,-1,-1 };	int nbv12 = 5;
static int n13[MAX_VOISIN] = { 0, 1, 6, 8,12,14,-1,-1,-1,-1,-1 };	int nbv13 = 6;
static int n14[MAX_VOISIN] = { 0, 1, 3,13,16,17,-1,-1,-1,-1,-1 };	int nbv14 = 6;
static int n15[MAX_VOISIN] = { 1, 2, 4,10,11,-1,-1,-1,-1,-1,-1 };	int nbv15 = 5;
static int n16[MAX_VOISIN] = { 0, 3, 8,14,17,-1,-1,-1,-1,-1,-1 };	int nbv16 = 5;
static int n17[MAX_VOISIN] = { 0, 3, 5, 14,16,-1,-1,-1,-1,-1,-1 };	int nbv17 = 5;

//neighbour weight (-1 == infinity)
static double w0[MAX_VOISIN] = { 1.20, 1.10, 0.60, 1.19, 0.85, 0.50, 0.30, 0.22, 0.45,-1,-1 };
static double w1[MAX_VOISIN] = { 1.20, 1.10, 1.80, 1.52, 1.05, 0.30, 0.50, 0.85, 1.19, 0.40 };
static double w2[MAX_VOISIN] = { 1.10, 0.92, 0.72, 0.42, 0.80, 0.78,-1,-1,-1,-1,-1 };
static double w3[MAX_VOISIN] = { 1.10, 0.92, 0.72, 0.80, 1.02, 0.73,-1,-1,-1,-1,-1 };
static double w4[MAX_VOISIN] = { 0.92, 0.95, 0.58, 0.46,-1,-1,-1,-1,-1,-1,-1 };
static double w5[MAX_VOISIN] = { 0.92,0.45,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
static double w6[MAX_VOISIN] = { 0.36, 0.36,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
static double w7[MAX_VOISIN] = { 0.72,0.72,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
static double w8[MAX_VOISIN] = { 0.60, 1.80, 1.06, 0.43,-1,-1,-1,-1,-1,-1,-1 };
static double w9[MAX_VOISIN] = { 1.52, 0.95, 0.47,1.27,-1,-1,-1,-1,-1,-1,-1 };
static double w10[MAX_VOISIN] = { 1.05, 0.42, 0.58, 0.47, 0.81, 0.66,-1,-1,-1,-1,-1 };
static double w11[MAX_VOISIN] = { 1.19, 0.30, 0.80,1.27, 0.81, 0.35, 0.25,-1,-1,-1,-1 };
static double w12[MAX_VOISIN] = { 0.85, 0.50, 0.36, 0.35, 0.40,-1,-1,-1,-1,-1,-1 };
static double w13[MAX_VOISIN] = { 0.50, 0.85, 0.36, 1.06, 0.40, 0.35,-1,-1,-1,-1,-1 };
static double w14[MAX_VOISIN] = { 0.30, 1.19, 0.80, 0.35, 0.32, 0.27,-1,-1,-1,-1,-1 };
static double w15[MAX_VOISIN] = { 0.40, 0.78, 0.46, 0.66, 0.25,-1,-1,-1,-1,-1,-1 };
static double w16[MAX_VOISIN] = { 0.22, 1.02, 0.43, 0.32, 0.30,-1,-1,-1,-1,-1,-1 };
static double w17[MAX_VOISIN] = { 0.45, 0.73, 0.45, 0.27, 0.30,-1,-1,-1,-1,-1,-1 };

static int *node_list_voisin[NB_NODE] = { n0,n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15,n16,n17 };
static int node_nb_voisin[NB_NODE] = { nbv0,nbv1,nbv2,nbv3,nbv4,nbv5,nbv6,nbv7,nbv8,nbv9,nbv10,nbv11,nbv12,nbv13,nbv14,nbv15,nbv16,nbv17 };
static double *node_weight_edge[NB_NODE] = { w0,w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12,w13,w14,w15,w16,w17 };

// lucie
//===============================================OBSTACLES================================================================
// Position of the obstacles'angle in m

/*//murs intérieurs
static double wall_yt[4][2] = { {-1.000, 0.850}, {-1.000, 0.830}, {-0.500, 0.830}, {-0.500, 0.850} }; // number 1
static double wall_yb[4][2] = { { 0.500,-1.500}, { 0.500,-1.000}, { 0.520,-1.000}, { 0.520,-1.500} }; // number 2
static double wall_bt[4][2] = { { 0.500, 1.000}, { 0.500, 1.500}, { 0.520, 1.500}, { 0.520, 1.000} }; // number 3
static double wall_bb[4][2] = { {-1.000,-0.850}, {-1.000,-0.830}, {-0.500,-0.830}, {-0.500,-0.850} }; // number 4

//mur central
static double wall_red_[12][2] = { {-0.200,0.400}, {0.200, 0.400}, {0.200, 0.300}, {-0.100, 0.300}, {-0.100, -0.300}, {0.200, -0.300}, 
								  {0.200, -0.400}, {-0.200, -0.400}, {-0.200, -0.100}, {-0.500, -0.100}, {-0.500, 0.100}, {-0.200, 0.100} }; // number 5

//murs exterieurs
static double wall_t[4][2] = { {-1.000, 1.500}, { 1.000, 1.500}, { 1.000,-1.500}, {-1.000,-1.500} }; // number6
*/

// corners' positions in m
static double corner_posx[NB_CORNER] = {-1.000,-1.000,-0.500,-0.500,   0.500, 0.500, 0.480, 0.480,  0.500,0.500,0.480,0.480,  -1.000,-1.000,-0.500,-0.500,  -0.200,0.200,0.200,-0.100,-0.100, 0.200, 0.200,-0.200,-0.200,-0.500,-0.500,-0.200,  -1.000,1.000, 1.000,-1.000 };
static double corner_posy[NB_CORNER] = { 0.850, 0.830, 0.830, 0.850,  -1.500,-1.000,-1.000,-1.500,  1.000,1.500,1.500,1.000,  -0.850,-0.830,-0.830,-0.850,   0.400,0.400,0.300, 0.300,-0.300,-0.300,-0.400,-0.400,-0.100,-0.100, 0.100, 0.100,   1.500,1.500,-1.500,-1.500 };

//number of corners
int nb_corner[NB_OBSTACLE] = { 4, 4, 4, 4, 12, 4 };

/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */
PathPlanning* init_path_planning()
{
	PathPlanning *path;
	int i = 0, j = 0, k=0, l=0, nb_corner_prev=0;

	// memory allocation
	path = (PathPlanning*) malloc(sizeof(PathPlanning));

	// ----- path-planning initialization start ----- //
	
	path->arrivedAtNode = false;
	path->next_node_index = 0;
	path->path_created = false;
	path->path_finalNode = false;
	path->path_baseNode = false;
	path->last_t=-15;
	path->last_ta=-15;
	path->int_error_angle=0;
	path->nodeNumber = (int*)malloc(NB_NODE * sizeof(int));

	// initialize all obstacles
	for (k = 0; k<NB_OBSTACLE; k++)
	{
		path->obstacle_list[k] = (Obstacle*)malloc(sizeof(Obstacle));
		path->obstacle_list[k]->number = k;
		path->obstacle_list[k]->nb_corner = nb_corner[k];

		for (l = 0; l < nb_corner[k]; l++) // enter the position of the corners for every obstacles
		{
			path->obstacle_list[k]->corner_posx[l] = corner_posx[nb_corner_prev];
			path->obstacle_list[k]->corner_posy[l] = corner_posy[nb_corner_prev];
			
			// length calculation for each segment of the obstacle
			if (l == nb_corner[k] - 1)
			{
				path->obstacle_list[k]->length[l] = sqrt(pow((corner_posx[nb_corner_prev] - corner_posx[nb_corner_prev-l]), 2) + pow((corner_posy[nb_corner_prev] - corner_posy[nb_corner_prev-l]), 2)); // when we arrive at the end of the corner table, we use the last corner and the first one to calculate the last length
			}
			else
			{
				path->obstacle_list[k]->length[l] = sqrt(pow((corner_posx[nb_corner_prev] - corner_posx[nb_corner_prev + 1]), 2) + pow((corner_posy[nb_corner_prev] - corner_posy[nb_corner_prev+1]), 2)); // basic calculation for a vector's length determined by two points
			}
			nb_corner_prev++;

		}
	}

	// initialize all nodes
	for (i = 0; i<NB_NODE; i++) {
		path->nodeslist[i] = (Node*)malloc(sizeof(Node));
		path->nodeslist[i]->numero = i;
		path->nodeslist[i]->posx = (double)node_x[i]/1000;
		path->nodeslist[i]->posy = (double)node_y[i]/1000;
		path->nodeslist[i]->nb_voisin = node_nb_voisin[i];

		for (j = 0; j < MAX_VOISIN; j++) {
			path->nodeslist[i]->liste_voisin[j] = node_list_voisin[i][j];
			path->nodeslist[i]->weight_edge[j] = node_weight_edge[i][j];
			path->nodeslist[i]->accessible[j] = true;
		}
		
		if (i < IND_PT_1) {
			path->nodeslist[i]->base = false;
			path->nodeslist[i]->inter = false;
			path->nodeslist[i]->weight = 1;
			path->nodeslist[i]->taken = false;
		}
		else if (i < IND_PT_2) {
			path->nodeslist[i]->base = false;
			path->nodeslist[i]->inter = false;
			path->nodeslist[i]->weight = 2;
			path->nodeslist[i]->taken = false;
		}
		else if (i < IND_PT_3) {
			path->nodeslist[i]->base = false;
			path->nodeslist[i]->inter = false;
			path->nodeslist[i]->weight = 3;
			path->nodeslist[i]->taken = false;
		}
		else if (i < IND_BASE) {
			path->nodeslist[i]->base = true;
			path->nodeslist[i]->inter = false;
			path->nodeslist[i]->weight = 0;
			path->nodeslist[i]->taken = true;
		}
		else if (i < IND_DRIVING) {
			path->nodeslist[i]->base = false;
			path->nodeslist[i]->inter = true;
			path->nodeslist[i]->weight = 0;
			path->nodeslist[i]->taken = true;
		}
		else {
			printf("error at path_planning\n");
		}
		
	}

	// ----- path-planning initialization end ----- //
	
	// return structure initialized
	return path;
}

/*! \brief change the y coordinate of nodes for yellow team
 *
 * \param[in,out] cvs controller main structure
 *
 */
void path_planning_correct_init(CtrlStruct *cvs) {
	PathPlanning *path=cvs->path;
	for (int i = 0; i < NB_NODE; i++) {
		path->nodeslist[i]->posy = -path->nodeslist[i]->posy;
	}
	return;
}

/*! \brief close the path-planning algorithm (memory released)
 * 
 * \param[in,out] path path-planning main structure
 */
void free_path_planning(PathPlanning *path)
{
	// ----- path-planning memory release start ----- //

	// free node structure
	for (int i = 0; i < NB_NODE; i++) {
		free(path->nodeslist[i]);
	}

	// free obstacle structure
	for (int i = 0; i < NB_OBSTACLE; i++) {
		free(path->obstacle_list[i]);
	}
	// ----- path-planning memory release end ----- //

	free(path);
}

// check if segment intersects obstacles
// return true if no intersection
// return false otherwise
// x,y = initial point of edge
// px, py = final point of edge
bool can_reach(double x, double y, double px, double py, PathPlanning* path) {
	int i, j, nb_corner;
	float par_r, par_s, den;								// [par_r, par_s] fraction of segment of edge until intersection
	float edge_start[SPACE_DIMENSION], edge_end[SPACE_DIMENSION];

	for (i = 0; i < NB_OBSTACLE; i++) {						// loop through all obstacles
		nb_corner = path->obstacle_list[i]->nb_corner;
		for (j = 0; j < nb_corner; j++) {					// loop through all edges
			edge_start[P_X] = path->obstacle_list[i]->corner_posx[j];
			edge_start[P_Y] = path->obstacle_list[i]->corner_posy[j];
			edge_end[P_X] = path->obstacle_list[i]->corner_posx[(j + 1) % nb_corner];
			edge_end[P_Y] = path->obstacle_list[i]->corner_posy[(j + 1) % nb_corner];

			den = (px - x)*(edge_end[P_Y] - edge_start[P_Y]) - (py - y)*(edge_end[P_X] - edge_start[P_X]);	//denominator
																											// if denominator == 0, segment and edges are parallel, so no intersection in most cases.
			if (den != 0) {
				par_r = ((y - edge_start[P_Y])*(edge_end[P_X] - edge_start[P_X]) - (x - edge_start[P_X])*(edge_end[P_Y] - edge_start[P_Y])) / den;
				par_s = ((y - edge_start[P_Y])*(px - x) - (x - edge_start[P_X])*(py - y)) / den;
				if (par_s >= 0 && par_s <= 1 && par_r >= 0 && par_r <= 1)	// if r AND s in [0;1], intersection the 2 lines is part of the segment
					return false;
			}
		}
	}
	return true;
}

// find the nearest node from point (x, y), and return its index
static int nearest_node(double x, double y, PathPlanning* all_nodes) {
	int nb_nd = -1;  // index of the node    
	int i;
	double dx;       // horizontal distance
	double dy;       // vertical distance
	double dist; // total distance
	double sdist = -1;   // shortest ditance yet. equal to -1 if no distance yet
	for (i = 0; i < NB_NODE; i++) {              // calculate the distance for all nodes
		dx = all_nodes->nodeslist[i]->posx - x;
		dy = all_nodes->nodeslist[i]->posy - y;
		dist = dx*dx + dy*dy;
		if (dist == 0) return i;                // if point (x, y) is on node, return the index of the node /// TODO see behaviour if ignore
		if (dist < sdist || sdist < 0) {
			if (can_reach(x, y, all_nodes->nodeslist[i]->posx, all_nodes->nodeslist[i]->posy, all_nodes)) {
				sdist = dist;
				nb_nd = i;
			}
		}
	}

	return nb_nd;
}

// find the optimal path through node from posx,posy ( inital pos) to x,y ( target position)
int* find_opt_path(double x, double y, double posx, double posy, CtrlStruct* cvs, double *dist) {

	printf("find opt path\n");
	PathPlanning* all_nodes = cvs->path;

	path_reset_accessible_node(cvs);
	path_update_accessible_node(cvs);

	// init parameters
	static int list_index[NB_NODE];                     // list of index of nodes from optimal path
														// from start to finish, then -1 to complete array
	int previous_node[NB_NODE];                         // nodes from the previous state of the algo
	double node_weight[NB_NODE];                         // distance to reach node from finish. -1 is infinite
	int i, j, k, l;
	int list[NB_NODE];                                  // temporary list of neighbours
	int finish = nearest_node(x, y, all_nodes);         // index of the last node
	int start = nearest_node(posx, posy, all_nodes);    // index of the first node
	if (start == -1 || finish == -1) {					// if start or finish is unreachable, return only -1,
		for (i = 0; i < NB_NODE; i++) {					// as no path can be found
			list_index[i] = -1;
		}
		return list_index;
	}
	if (start == finish) {                              // if start and finish are same node, return only start index
		list_index[0] = start;
		for (i = 1; i < NB_NODE; i++) {
			list_index[i] = -1;
		}
		return list_index;
	}


	int neighbours[NB_NODE];                            // how many edges btw a node and the finish node. -1 if not in a possible path yet
	int index;                                          // temporary save for index
	for (i = 0; i < NB_NODE; i++) {                      // init neighbours and node_weight
		neighbours[i] = -1;
		node_weight[i] = -1.;
	}
	neighbours[finish] = 0;                             // node finish is the first
	node_weight[finish] = 0;                            // need 0 distance to reach finish from finish
	previous_node[0] = finish;                          // first step init with only finish node
	int next_num = 1;                                   // number of nodes in current step

														// search relation between nodes
	for (i = 1; i < NB_NODE; i++) {                      // loop after each step
		l = 0;
		for (k = 0; k < next_num; k++) {             // loop through each node in the current step
			for (j = 0; j < all_nodes->nodeslist[previous_node[k]]->nb_voisin; j++) {  // loop through each neighbour of the current node
																					   // check if neighbour is accessible
				if (all_nodes->nodeslist[i]->accessible[j]) {
					index = all_nodes->nodeslist[previous_node[k]]->liste_voisin[j];      // index of the current neighbour
																						  // if actual weight is higher than th new possible weight, update data
					if (((node_weight[previous_node[k]] + all_nodes->nodeslist[previous_node[k]]->weight_edge[j]) < node_weight[index]) || node_weight[index] == -1.) {                  // if node not already visited, update information
						neighbours[index] = i;
						node_weight[index] = node_weight[previous_node[k]] + all_nodes->nodeslist[previous_node[k]]->weight_edge[j];
						list[l] = index;
						l++;
					}
				}
			}
		}
		if (node_weight[start] != -1.) break;   // if start node visited, end of loop
		next_num = l;							// number of nodes in current step
		for (l = 0; l < next_num; l++) {		// transfer temporary list of nodes for the next step
			previous_node[l] = list[l];
		}
	}

	// find and sort nodes from path with least number of connection
	list_index[0] = start;  // first index is the start node
	index = start;          // current node
	for (i = 1; i < neighbours[start] + 1; i++) {                              // increment each steps until
		for (j = 0; j < all_nodes->nodeslist[index]->nb_voisin; j++) {         // search through all neighbours the correct node
																			   // check if step of neighbour is 1 step lower than current node
			if (all_nodes->nodeslist[i]->accessible[j]) {
				if (neighbours[all_nodes->nodeslist[index]->liste_voisin[j]] == neighbours[start] - i) {
					//check if weight of neighbour is lower than weight of node
					if (node_weight[all_nodes->nodeslist[index]->liste_voisin[j]] < node_weight[index]) {
						// check if diff between the 2 weights equal the weight of the edge btw them
						if (fabs(node_weight[all_nodes->nodeslist[index]->liste_voisin[j]] - node_weight[index] + all_nodes->nodeslist[index]->weight_edge[j]) < TOL) {
							list_index[i] = all_nodes->nodeslist[index]->liste_voisin[j]; // store in current step the correct node
							index = list_index[i]; // correct node becomes next node
							break;
						}
					}
				}
			}
		}
	}
	// fill the other part of array with -1
	for (i = neighbours[start] + 1; i < NB_NODE; i++) {
		list_index[i] = -1;
	}
	*dist = node_weight[start];		// return the total weight of the path
	return list_index;				// return the indexes of the nodes in correct order
}

// high level decision making about which target to go to
void path_decision_making(CtrlStruct* cvs) {
	printf("path_decision_making\n");
	// if carrying 2 targets, then go to target base
	if (cvs->inputs->nb_targets == 2) {
		objectif_target_base(cvs);
		printf("going to target base\n");
		return;
	}

	// run objectif target : if target available, then go to it, else (remaining target == 0), then go to base
	if (objectif_closest_target(cvs)==0) {
		// control if robot is at a base
		check_at_base(cvs);
		// if at base
		if (cvs->strat->at_base) {
			cvs->strat->main_state = GAME_STATE_FINISH;
			cvs->strat->going_base = false;
		}
		// if not at base
		else {
			// set the objectif to closest base
			objectif_closest_base(cvs);
			cvs->strat->going_base = true;
		}
	}

	return;
}

// return remaining target and set the objectif to the closest target if there is remaining target
static int objectif_closest_target(CtrlStruct* cvs) {
	int i = 0, nodeIndex = 0, targetRemaining = 0;
	int* opt_path_index;
	double weight_max=0, weight_node=0, weight_ratio=0, d = 0;
	double kal_x, kal_y;

	PathPlanning* path = cvs->path;

	// kalman position
	kal_x = cvs->kalman->kalman_pos.x();
	kal_y = cvs->kalman->kalman_pos.y();

	// for all node
	for (i = 0; i < NB_NODE; i++) {
		// if node not taken
		if (!(path->nodeslist[i]->taken)) {
			// target remaining plus 1
			targetRemaining++;

			// optimal path to node
			opt_path_index = find_opt_path(path->nodeslist[i]->posx, path->nodeslist[i]->posy, kal_x,kal_y, cvs, &d);
			if (opt_path_index[0] < 0) {
				printf("objectif_closest_target : path not valid \n");
				system("PAUSE");
				exit(EXIT_FAILURE);
			}

			// weighting the point based on the distance and the point weight
			weight_node = path->nodeslist[i]->weight;
			weight_ratio = pow(weight_node, 1.2) / pow(d,2);
			
			// look for target with higher weight ratio 
			if (weight_max < weight_ratio) {
				// store new weight and index of the new closest node
				weight_max = weight_ratio;
				nodeIndex = i;
			}
		}
	}

	// if remains some not taken target
	if (targetRemaining > 0) {
		cvs->strat->objectif_x = path->nodeslist[nodeIndex]->posx;
		cvs->strat->objectif_y = path->nodeslist[nodeIndex]->posy;
		cvs->strat->main_state = GAME_STATE_FIND_PATH;
		cvs->strat->nb_target_before = cvs->inputs->nb_targets;
		cvs->strat->going_base = false;
	}
	printf("path obj target : nodeIndex to go = %d\n", nodeIndex);

	return targetRemaining;
}

// find closest base
static void objectif_closest_base(CtrlStruct* cvs) {
	printf("objectif_closest_base\n");
	int i = 0, nodeIndex = 0, targetRemaining = 0;
	double dmin = 4, d = 0, dx = 0, dy = 0;
	double kal_x, kal_y;

	PathPlanning* path = cvs->path;

	kal_x = cvs->kalman->kalman_pos.x();
	kal_y = cvs->kalman->kalman_pos.y();

	// for all node
	for (i = 0; i < NB_NODE; i++) {
		// if node is a base node
		if (path->nodeslist[i]->base) {

			// distance to base
			dx = path->nodeslist[i]->posx - kal_x;
			dy = path->nodeslist[i]->posy - kal_y;
			d = sqrt(dx*dx + dy*dy);

			// look for min dist between all base
			if (d < dmin) {
				// store new min distance and index of the new closest node
				dmin = d;
				nodeIndex = i;
			}
		}
	}

	printf("path obj base nodeIndex=%d\n", nodeIndex);
	cvs->strat->objectif_x = path->nodeslist[nodeIndex]->posx;
	cvs->strat->objectif_y = path->nodeslist[nodeIndex]->posy;
	cvs->strat->main_state = GAME_STATE_FIND_PATH;
	cvs->strat->going_base = true;
	return;
}

// set objectif to target base
static void objectif_target_base(CtrlStruct* cvs) {
	printf("objectif_target_base\n");
	cvs->strat->objectif_x = cvs->path->nodeslist[TARGET_BASE_NODE_INDEX]->posx;
	cvs->strat->objectif_y = cvs->path->nodeslist[TARGET_BASE_NODE_INDEX]->posy;
	cvs->strat->main_state = GAME_STATE_FIND_PATH;
	cvs->strat->going_base = true;
	return;
}

// check if the robot is near a base node
static void check_at_base(CtrlStruct* cvs) {
	int i = 0, nodeIndex = 0, targetRemaining = 0;
	double dmin = 4, d = 0, dx = 0, dy = 0;
	double kal_x, kal_y;

	PathPlanning* path = cvs->path;

	kal_x = cvs->kalman->kalman_pos.x();
	kal_y = cvs->kalman->kalman_pos.y();

	cvs->strat->at_base = false;

	// for all node
	for (i = 0; i < NB_NODE; i++) {
		// if node is a base node
		if (path->nodeslist[i]->base) {

			// distance to base
			dx = path->nodeslist[i]->posx - kal_x;
			dy = path->nodeslist[i]->posy - kal_y;
			d = sqrt(dx*dx + dy*dy);

			// if close to base node, then consider it as at base
			if (d < TRAJET_MIN) {
				cvs->strat->at_base = true;
				return;
			}
		} // end base node
	} // end for

	return;
}

// update the taken target parameters while driving
void path_update_taken(CtrlStruct* cvs) {
	int i = 0;
	double delta_trajet = 0, dx = 0, dy = 0;
	PathPlanning* path = cvs->path;

	// for all node
	for (i = 0; i < NB_NODE; i++) {
		// if node is not taken
		if (!(path->nodeslist[i]->taken)) {			
			// calculate the distance to the node
			dx = path->nodeslist[i]->posx - cvs->kalman->kalman_pos.x();
			dy = path->nodeslist[i]->posy - cvs->kalman->kalman_pos.y();
			delta_trajet = sqrt(dx*dx + dy*dy);

			// if on node
			if (delta_trajet < TRAJET_MIN) {
				// if detected by inputs
				if (cvs->inputs->target_detected) {
					path->nodeslist[i]->taken = false;
				}
				// not detected
				else {
					path->nodeslist[i]->taken = true;
				}
			}
		}

	}
	//cvs->path->nodeslist[1]->taken;
	return;

}

// update the accesible nodes with respect to the opponent position
static void path_update_accessible_node(CtrlStruct* cvs) {

	int n = 0, start = 0, end = 0, nb_neig = 0;
	double circle_opp[C_NB_DATA];
	double edge_start[SPACE_DIMENSION], edge_end[SPACE_DIMENSION];
	double length;
	int index_neig = 0; // neighbour index
	bool inter = false;
	
	// for each opponent
	for (n = 0; n < cvs->nb_opp; n++) {
		// set circle array
		circle_opp[C_X] = cvs->opp_pos->x[n];
		circle_opp[C_Y] = cvs->opp_pos->y[n];
		circle_opp[C_R] = R_OPP;

		// for all nodes i
		for (start = 0; start < NB_NODE; start++) {
			// set the start edge point of node
			edge_start[P_X] = cvs->path->nodeslist[start]->posx;
			edge_start[P_Y] = cvs->path->nodeslist[start]->posy;

			// get the number of neighbours that start has
			nb_neig = cvs->path->nodeslist[start]->nb_voisin;

			// for all neighbours of start
			for (end = 0; end < nb_neig; end++) {
				// get the index of the neighbours with respect to node start
				index_neig = cvs->path->nodeslist[start]->liste_voisin[end];

				// set the end edge point of node
				edge_end[P_X] = cvs->path->nodeslist[index_neig]->posx;
				edge_end[P_Y] = cvs->path->nodeslist[index_neig]->posy;

				// get the length between the 2 nodes
				length = cvs->path->nodeslist[start]->weight_edge[end];

				// compute intersection between opponent circle and edge of 2 nodes
				inter=inter_edge_circle(circle_opp, edge_start, edge_end, length);

				// if there is an intersection, then the node neighbour is not accessible
				if (inter) {
					cvs->path->nodeslist[start]->accessible[end] = false;
				}
			}
		}
	}
	
}

// reset all node to accessible
static void path_reset_accessible_node(CtrlStruct* cvs) {
	int i = 0, j = 0, nb_neig = 0;
	// for all nodes
	for (i = 0; i < NB_NODE; i++) {
		// get the number of neighbours
		nb_neig = cvs->path->nodeslist[i]->nb_voisin;

		// for all neighbours of i
		for (j = 0; j < nb_neig; j++) {
			// set the accessibility from i to j to true
			cvs->path->nodeslist[i]->accessible[j] = true;
		}
	}
	return;
}


NAMESPACE_CLOSE();
