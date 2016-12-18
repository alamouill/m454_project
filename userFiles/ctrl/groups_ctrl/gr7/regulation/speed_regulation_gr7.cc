/* Mobile Robots Project - speed_regulation_gr7.cc
 * brief speed regulation
 * Version 1.8
 * Last Update: 18/12/16
 * Group 7: Baumann, El-Hamamsy, Laumouille, Triquet
 * EPFL, MT*/ 

#include "speed_regulation_gr7.h"
#include "useful_gr7.h"

NAMESPACE_INIT(ctrlGr7);

/*! \brief wheel speed regulation
 * 
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	// variables declaration
	
	static double int_valr = 0;
	static double int_vall = 0;
	static double var_r;
	static double var_l;
	static int dejtouch = 0;
	double r_sp, l_sp;
	double dt;
	double kp = 50; //50
	double ki = 250;//75

	CtrlIn *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg = cvs->sp_reg;

	// wheel speeds
	r_sp = inputs->r_wheel_speed;
	l_sp = inputs->l_wheel_speed;

	// time
	dt = inputs->t - sp_reg->last_t; // time interval since last call

	// ----- Wheels regulation computation start ----- //
	if (!(inputs->u_switch[0] || inputs->u_switch[1]))
	{
		int_valr += (r_sp_ref - r_sp)*dt;
		int_vall += (l_sp_ref - l_sp)*dt;
		int_vall = limit_range(int_vall, -50, 50);
		int_valr = limit_range(int_valr, -50, 50);

	}
	
	var_r = kp*(r_sp_ref - r_sp) + ki*int_valr;
	var_l = kp*(l_sp_ref - l_sp) + ki*int_vall;	

	// wheel commands
	
	outputs->wheel_commands[R_ID] = var_r;
	outputs->wheel_commands[L_ID] = var_l;

	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
}

NAMESPACE_CLOSE();
