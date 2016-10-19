#include "mcl_gr7.h"
#include "odometry_gr7.h"
#include "triangulation_gr7.h"
#include "useful_gr7.h"

NAMESPACE_INIT(ctrlGr7);

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
