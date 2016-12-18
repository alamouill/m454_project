#include "init_pos_gr6.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr6);
#define DEG_TO_RAD (M_PI/180.0)
#define RAD_2_DEG 1/DEG_TO_RAD

/*! \brief set the initial robot position
 * 
 * \param[in] robot_id robot ID
 * \param[out] rob_pos robot position structure
 *
 * Adapt these initial positions, depending on the game map.
 * blue_T1 : 0.67
 * blue_T2 : 1.15
 * blue_R3 : -90.0
 */
void set_init_position(int robot_id, RobotPosition *rob_pos)
{
	switch (robot_id)
	{
		case ROBOT_B: // blue robot
			rob_pos->x = 0.67;
			rob_pos->y = 1.15;
			rob_pos->theta = -90.0;
			break;

		case ROBOT_R: // red robot
			rob_pos->x = 0.82;
			rob_pos->y = 1.4;
			rob_pos->theta = -90.0;
			break;

		case ROBOT_Y: // yellow robot
			rob_pos->x = 0.67;
			rob_pos->y = -1.15;
			rob_pos->theta = 90.0;
			break;

		case ROBOT_W: //  white robot
			rob_pos->x = 0.82;
			rob_pos->y = -1.4;
			rob_pos->theta = 90.0;
			break;
	
		default:
			printf("Error: unknown robot ID: %d !\n", robot_id);
			exit(EXIT_FAILURE);
	}		
}

NAMESPACE_CLOSE();
