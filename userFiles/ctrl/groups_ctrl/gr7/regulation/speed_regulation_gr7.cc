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
	static double int_valr=0;
	static double int_vall = 0;
	static double var_r;
	static double var_l;
	double r_sp, l_sp;
	double dt;
	double kp = 16.00177;
	double ki = 2.0011;

	// variables declaration
	CtrlIn *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg  = cvs->sp_reg;

	// wheel speeds
	r_sp = inputs->r_wheel_speed;
	l_sp = inputs->l_wheel_speed;

	// time
	dt = inputs->t - sp_reg->last_t; // time interval since last call

	// ----- Wheels regulation computation start ----- //
	int_valr += (r_sp_ref - r_sp)*dt;
	int_valr = limit_range(int_valr, -100, 100);
	
	int_vall += (l_sp_ref - l_sp)*dt;
	int_vall = limit_range(int_vall, -100, 100);
	
	var_r = kp*(r_sp_ref - r_sp) + ki*int_valr;
	var_l = kp*(l_sp_ref - l_sp) + ki*int_vall;


	if (r_sp_ref == 0 && 0 == l_sp_ref)
	{
		outputs->wheel_commands[R_ID] = 0.0;
		outputs->wheel_commands[L_ID] = 0.0;

	}

	// wheel commands
	outputs->wheel_commands[R_ID] = var_r;
	outputs->wheel_commands[L_ID] = var_l;

	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
///	printf("time= %f \t r_speed volt =%f\n", inputs->t ,var_r);
}

NAMESPACE_CLOSE();
