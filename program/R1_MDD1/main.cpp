/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa404
  * @version V0.2.2
  * @date    25-June-2024
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
Pid mtr_pid[4];
int mtr_rq[4]; //モーター出力[% <95]
Motor mtr[4];
Encoder enc[4]; Encoder_data enc_data[4];

//MyUart<12> Uartdebug;
MyUart<12> uartDD;
union {float f[3]; uint8_t u8[12];} uartDD_raw;
float uartDD_data[3];
MyUart<12> uartSensor;
union {float f[3]; uint8_t u8[12];} uartSensor_data;

//run every 1ms
void main_interrupt(void) {
	//communicate
	if (uartDD.read(uartDD_raw.u8)) {
		uartDD_data[0] = uartDD_raw.f[0];
		uartDD_data[1] = uartDD_raw.f[1];
		uartDD_data[2] = uartDD_raw.f[2];
	}
	uartSensor.read(uartSensor_data.u8);
	uartDD.write(uartSensor_data.u8, 12);

	//read encoder
	for(int i = 0; i < 4; ++i) enc[i].interrupt(&enc_data[i]);

	if (uartDD_data[0]==0 && uartDD_data[1]==0 && uartDD_data[2]==0) {
		for (int i = 0; i < 4; ++i) {
			mtr_rq[i] = 0;
			mtr_pid[i].reset();
		}
	} else {
		rq_abs.x = uartDD_data[0]*cos(uartDD_data[1]);
		rq_abs.y = uartDD_data[0]*sin(uartDD_data[1]);
		rq_theta = uartDD_data[2];

		//localize rq
		rq_local = Point::rotated(rq_abs, -uartSensor_data.f[2]);

		//communicate with motor
		double targetByRot = rq_theta;
		mtr_target[0] = Point::rotated(rq_local, 45).x*M_SQRT2+targetByRot;
		mtr_target[1] = Point::rotated(rq_local, -45).x*M_SQRT2+targetByRot;
		mtr_target[2] = Point::rotated(rq_local, -135).x*M_SQRT2+targetByRot;
		mtr_target[3] = Point::rotated(rq_local, 135).x*M_SQRT2+targetByRot;
		//mtr_write
		for (int i = 0; i < 4; ++i) {
			mtr_rq[i] = mtr_pid[i].control(mtr_target[i],enc_data[i].volcity,1);
		}
	}
	for(int i = 0; i < 4; ++i) {
		if (mtr_rq[i] > 95) mtr_rq[i] = 95;
		if (mtr_rq[i] < -95) mtr_rq[i] = -95;
		mtr[i].write(mtr_rq[i]);
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

	for (int i = 0; i < 4; ++i) mtr_pid[i].setGain(0.1,0,1,20);

	//Uartdebug.init(USB_miniB,9600);
	uartDD.init(USB_B,115200);
	uartSensor.init(USB_A,115200);

	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {}
}
