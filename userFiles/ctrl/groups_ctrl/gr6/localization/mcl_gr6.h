/*! 
 * \author Group 6
 * \file mcl_gr6.h
 * \brief localization using Monte Carlo technic
 */

#ifndef _MCL_GR6_H_
#define _MCL_GR6_H_

#include "CtrlStruct_gr6.h"
#include "init_pos_gr6.h"

NAMESPACE_INIT(ctrlGr6);

/// Markov main structure
struct MCLStruct
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void mcl_update(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
