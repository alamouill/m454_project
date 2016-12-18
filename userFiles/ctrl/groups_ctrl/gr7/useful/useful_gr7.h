/*! 
 * \author Group 7
 * \file useful_gr7.h
 * \brief useful functions to use in the controller
 */

#ifndef _USEFUL_GR7_H_
#define _USEFUL_GR7_H_

#include "namespace_ctrl.h"
#include <vector>

NAMESPACE_INIT(ctrlGr7);

double rnd();
double norm_dist(double dx, double dy);
double limit_range(double x, double min, double max);
double limit_angle(double x);
double first_order_filter(double last_val, double new_val, double tau, double delta_t);
double sgn(double x);
double get_Distance(double pos[2], double goal[2]);
double get_Distance(std::pair<int, int> point1, std::pair<int, int> point2);

NAMESPACE_CLOSE();

#endif
