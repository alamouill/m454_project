/* Mobile Robots Project - mcl_gr7.h
 * Version 0
 * Last Update: 18/10/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 

#ifndef _MCL_GR7_H_
#define _MCL_GR7_H_

#include "CtrlStruct_gr7.h"
#include "init_pos_gr7.h"

NAMESPACE_INIT(ctrlGr7);

/// Markov main structure
struct MCLStruct
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void mcl_update(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
