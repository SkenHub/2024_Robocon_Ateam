#ifndef PID_H_
#define PID_H_

enum class ControlType
{
	P,
	I,
	D
};

class Pid
{
public:
	Pid(void);
	void setGain(double p_gain, double i_gain, double d_gain, double time_constant = 0);
	double control(double target, double now, int control_period);
	double control(double e, int control_period);
	void reset(void);
	double getControlValue(ControlType control_type);

private:
	double integral_error_;
	double before_error_;
	double kp_;
	double ki_;
	double kd_;
	double d_error_;
	double p_control_;
	double i_control_;
	double d_control_;
	double time_constant_;
};

#endif /* PID_H_ */
