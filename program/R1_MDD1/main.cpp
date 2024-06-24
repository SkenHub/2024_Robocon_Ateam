/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa404
  * @version V0.2.1
  * @date    23-June-2024
  * @brief   R1 MDD1(undercarriage) function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

#include <cmath>

using std::sin;
using std::cos;

constexpr double TIRE_DIAMETER = 100; //駆動輪直径[mm]
constexpr double BODY_DIAMETER = 1444.616; //駆動輪間直径[mm]
constexpr uint8_t FROM_DD_DATASIZE = 12;
constexpr uint8_t FROM_SENSOR_DATASIZE = 12;
constexpr double Kp = 0.1;
constexpr double Ki = 0;
constexpr double Kd = 1;

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

Point rq_abs; double rq_theta; //絶対座標系での指定{{x[mm/s],y[mm/s]},theta[deg]}
Point rq_local;
double mtr_target[4]; //モーター目標速度[mm/s]
double mtr_rq[4] = {}; //モーター出力[% <95]
Motor mtr[4];
Encoder enc[4]; Encoder_data enc_data[4];

//MyUart<12> Uartdebug;
MyUart<FROM_DD_DATASIZE> uartDD;
union {float f[FROM_DD_DATASIZE/4]; uint8_t u8[FROM_DD_DATASIZE];} uartDD_data;
MyUart<FROM_SENSOR_DATASIZE> uartSensor;
union {float f[FROM_SENSOR_DATASIZE/4]; uint8_t u8[FROM_SENSOR_DATASIZE];} uartSensor_data;

double debug[8];
int is_read;
//run every 1ms
void main_interrupt(void) {
	//communicate
	is_read = uartDD.read(uartDD_data.u8);
	uartSensor.read(uartSensor_data.u8);
	uartDD.write(uartSensor_data.u8, FROM_SENSOR_DATASIZE);
	if (uartDD_data.f[0]==0 && uartDD_data.f[1]==0 && uartDD_data.f[2]==0) {

		for (int i = 0; i < 4; ++i) {
			mtr[i].write(0);
			mtr_rq[i] = 0;
		}

	} else {

	rq_abs.x = uartDD_data.f[0]*cos(uartDD_data.f[1]);
	rq_abs.y = uartDD_data.f[0]*sin(uartDD_data.f[1]);
	rq_theta = uartDD_data.f[2];

	//read encoder
	for(int i = 0; i < 4; ++i) enc[i].interrupt(&enc_data[i]);

	//localize rq
	rq_local = Point::rotated(rq_abs, -uartSensor_data.f[2]);
	debug[0] = rq_abs.x;
	debug[1] = rq_abs.y;
	debug[2] = rq_local.x;
	debug[3] = rq_local.y;

	//communicate with motor
	//unavailable: encoder control
	double targetByRot = rq_theta;
	mtr_target[0] = Point::rotated(rq_local, 45).x*M_SQRT2+targetByRot;
	mtr_target[1] = Point::rotated(rq_local, -45).x*M_SQRT2+targetByRot;
	mtr_target[2] = Point::rotated(rq_local, -135).x*M_SQRT2+targetByRot;
	mtr_target[3] = Point::rotated(rq_local, 135).x*M_SQRT2+targetByRot;
	//mtr_write
	for (int i = 0; i < 4; ++i) {
		mtr_rq[i] += (mtr_target[i]-enc_data[i].volcity)*Kp;
		if (mtr_rq[i] > 95) mtr_rq[i] = 95;
		if (mtr_rq[i] < -95) mtr_rq[i] = -95;
	}
	for(int i = 0; i < 4; ++i) mtr[i].write(static_cast<int>(mtr_rq[i]));

	}
}

//main
int main(void)
{
	sken_system.init();

	//左のコネクタから順番に
	mtr[0].init(Apin,B8,TIMER10,CH1);
	mtr[0].init(Bpin,B9,TIMER11,CH1);
	mtr[1].init(Apin,A6,TIMER13,CH1);
	mtr[1].init(Bpin,A7,TIMER14,CH1);
	mtr[2].init(Apin,A8,TIMER1,CH1);
	mtr[2].init(Bpin,A11,TIMER1,CH4);
	mtr[3].init(Apin,B14,TIMER12,CH1);
	mtr[3].init(Bpin,B15,TIMER12,CH2);

	enc[0].init(A0,A1,TIMER5,TIRE_DIAMETER);
	enc[1].init(B3,A5,TIMER2,TIRE_DIAMETER);
	enc[2].init(B6,B7,TIMER4,TIRE_DIAMETER);
	enc[3].init(C6,C7,TIMER3,TIRE_DIAMETER);

	//Uartdebug.init(USB_miniB,9600);
	uartDD.init(USB_B,115200);
	uartSensor.init(USB_A,115200);

	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {}
}
