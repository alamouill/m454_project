#include "speed_regulation_gr6.h"
#include "useful_gr6.h"

NAMESPACE_INIT(ctrlGr6);

/*! \brief wheel speed regulation
*
* \param[in,out] cvs controller main structure
* \parem[in] r_sp_ref right wheel speed reference [rad/s]
* \parem[in] l_sp_ref left wheel speed reference [rad/s]
*/
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	double r_sp, l_sp;
	double dt;

	// variables declaration
	CtrlIn *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;
	double K = 20; // gain du PI					fred kp 20, ki 100
	double Ki = 50; // constante de l'integrateur
	double u[2] = { 0 }; // consigne à appliquer
	double int_error_r;
	double int_error_l;

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

	//Integral term of the error (ARW)
	int_error_r = (sp_reg->int_error_r) + (r_sp_ref - r_sp)*dt;
	int_error_l = (sp_reg->int_error_l) + (l_sp_ref - l_sp)*dt;

	//Limitation of Integral term of the error (ARW)
	int_error_r = limit_range(int_error_r, -50, 50);
	int_error_l = limit_range(int_error_l, -50, 50);

	// Consigne to apply on the wheel speed command
	u[0] = K*(r_sp_ref - r_sp) + Ki*(int_error_r);
	u[1] = K*(l_sp_ref - l_sp) + Ki*(int_error_l);
	//printf("t=%f,\t u_r=%f\n", inputs->t, u[0]);

	// wheel commands
	outputs->wheel_commands[R_ID] = u[0];
	outputs->wheel_commands[L_ID] = u[1];

	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;

	// last update Integral term of the error
	sp_reg->int_error_r = int_error_r;
	sp_reg->int_error_l = int_error_l;

	return;
}

NAMESPACE_CLOSE();
