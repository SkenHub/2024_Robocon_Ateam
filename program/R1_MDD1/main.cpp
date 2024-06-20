/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa404
  * @version V0.0.0
  * @date    24-May-2024
  * @brief   R1 undercarriage function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

#include <cmath>

//#define ENABLE_SBDBT_PS3

using std::sin;
using std::cos;

constexpr const double SPD = 10.0;
constexpr const double TIRE_DIAMETER = 1; //[mm]
constexpr const double BODY_DIAMETER = 1; //[mm]

//このファイルの全ての角度(theta)は度数法。
class Point {
public:
	double x;
	double y;
	Point(double i=0, double j=0) : x(i),y(j) {}
	void rotate(double theta) {
		*this = rotated(*this, theta);
	}
	static Point rotated(Point i, double theta) {
		double rad = theta / 180 * M_PI;
		return Point(i.x*cos(rad)-i.y*sin(rad), i.x*sin(rad)+i.y*cos(rad));
	}
	Point operator=(Point i) {
		this->x = i.x;
		this->y = i.y;
		return *this;
	}
};

Point abs_rq_p; double rq_theta; //絶対座標系での指定{{x,y},theta}
Point local_rq_p;
double mtr_rq[4];
Motor mtr[4];
double direction = 0; //置く向きに合わせて初期値を変える。
Encoder enc[4]; Encoder_data enc_data[4];

#ifdef ENABLE_SBDBT_PS3
PS3 ps3;
PS3_data ps3_data;
void stick_resize(double&,double&);
#else
Gpio sw;
#endif

void get_rq(void);
void main_interrupt(void);

#ifdef ENABLE_SBDBT_PS3
//return -1~1
void stick_resize(double &x, double &y) {
	double longer = (abs(x) > abs(y) ? abs(x) : abs(y));
	double angle = std::atan2(y, x);

	x = cos(angle) * longer / 64;
	y = sin(angle) * longer / 64;
}
#endif

//communicate
void get_rq(void) {
	double rqx,rqy;

	/*   unavailable    */
	/*   coming soon    */
	/* work in progress */

#ifdef ENABLE_SBDBT_PS3
	ps3.Getdata(&ps3_data);
	rqx = ps3_data.LxPad;
	rqy = ps3_data.LyPad;
	stick_resize(rqx, rqy);
	rq_theta = ps3_data.RxPad/64*SPD;
#else
	//test
	rqx = (sw.read()?0:1);
	rqy = (sw.read()?1:0);
	rq_theta = 0;
#endif

	abs_rq_p.x = rqx*SPD;
	abs_rq_p.y = rqy*SPD;
}

double debug[8];
//run every 1ms
void main_interrupt(void) {
	//communicate
	get_rq();
	for(int i = 0; i < 4; ++i) enc[i].interrupt(&enc_data[i]);

	//localize rq
	//direction += static_cast<double>(enc_data[0][1]+enc_data[1][1]+enc_data[2][1])/(3.0*2160.0)*(TIRE_DIAMETER/BODY_DIAMETER)*360;
	local_rq_p = Point::rotated(abs_rq_p, -direction);
	debug[0] = local_rq_p.x;
	debug[1] = local_rq_p.y;

	//communicate with motor
	mtr_rq[0] = Point::rotated(local_rq_p, -45).x;
	mtr_rq[1] = Point::rotated(local_rq_p, 45).x;
	mtr_rq[2] = Point::rotated(local_rq_p, 135).x;
	mtr_rq[3] = Point::rotated(local_rq_p, -135).x;
	//mtr_write
	for(int i = 0; i < 4; ++i) mtr[i].write(static_cast<int>(mtr_rq[i]));
}

//main
int main(void)
{
	sken_system.init();

	mtr[0].init(Apin,B14,TIMER12,CH1);
	mtr[0].init(Bpin,B15,TIMER12,CH2);
	mtr[1].init(Apin,A8,TIMER1,CH1);
	mtr[1].init(Bpin,A11,TIMER1,CH4);
	mtr[2].init(Apin,A6,TIMER13,CH1);
	mtr[2].init(Bpin,A7,TIMER14,CH1);
	mtr[3].init(Apin,B8,TIMER10,CH1);
	mtr[3].init(Bpin,B9,TIMER11,CH1);

	enc[0].init(A0,A1,TIMER5,TIRE_DIAMETER);
	enc[1].init(B3,A5,TIMER2,TIRE_DIAMETER);
	enc[2].init(B6,B7,TIMER4,TIRE_DIAMETER);
	enc[3].init(C6,C7,TIMER3,TIRE_DIAMETER);

#ifdef ENABLE_SBDBT_PS3
	ps3.StartRecive();
#else
	sw.init(C13,INPUT_PULLUP);
#endif

	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {}
}
