#include "mcl_gr6.h"
#include "odometry_gr6.h"
#include "triangulation_gr6.h"
#include "useful_gr6.h"

NAMESPACE_INIT(ctrlGr6);

/*! \brief update using MCL technic
 * 
 * \param[in,out] cvs controller main structure
 */
void mcl_update(CtrlStruct *cvs)
{
	// variable declaration
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
}

NAMESPACE_CLOSE();
