/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa404
  * @version V0.3.7
  * @date    06-July-2024
  * @brief   R1 MDD1(undercarriage) function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"
#include "myuart.hpp"

#include <cmath>

//#define DD_DEBUG
#define DISABLE_PC

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
union f3_u812_convert {
	float f[3];
	uint8_t u8[12];
};

Point rq_abs; double rq_theta; //絶対座標系での指定{{x[mm/s],y[mm/s]},theta[deg]}
Point rq_local;
double mtr_target[4]; //モーター目標速度[deg/s]
Pid mtr_pid[4];
double mtr_rq[4]; //モーター出力[% <95]
Motor mtr[4];
Encoder enc[4]; Encoder_data enc_data[4];
double mtr_now[4]; //[deg/s]

MyUart<0> uartDD;
MyUart<12> uartSensor;

struct DataFromPC {
  float move_spd; //[mm/s]
  float move_dir; //[deg]
  float rot; //[deg/s]
  uint8_t field; //0~1
  uint8_t action; //0~5
} received;
struct DataSTM {
  float x; //[mm]
  float y; //[mm]
  float theta; //[deg]
  uint8_t field; //0~1
  uint8_t action; //0~5
} now;


template <uint8_t SIZE> class Average {
public:
	double BUFFER_[SIZE] = {};
	Average(void) = default;
	double culc(int x) {
		long double tmp = 0;
		for (int i = 0; i < SIZE-1; ++i) {
			BUFFER_[i] = BUFFER_[i+1];
			tmp += BUFFER_[i];
		}
		BUFFER_[SIZE-1] = x;
		return (tmp+x)/SIZE;
	}
};
Average<100> culc_ave[4];


#ifdef DISABLE_PC
Gpio sw;
#else
MyUart<14> uartPC;

float float_reverse(float in) {
	ConvertIntFloat trans, ans;
	trans.float_val = in;
	for (int i = 0; i < 4; ++i) ans.uint8_val[i] = trans.uint8_val[3-i];
	return ans.float_val;
}

void PC_receive(void) {
	uint8_t tmp[14];
	if (uartPC.read(tmp)) {
		f3_u812_convert conv;
		for (int i = 0; i < 12; ++i) conv.u8[i] = tmp[i];
		received.move_spd = float_reverse(conv.f[0]);
		received.move_dir = float_reverse(conv.f[1]);
		received.rot = float_reverse(conv.f[2]);
		received.field = tmp[12];
		received.action = tmp[13];

		now.field = received.field;
		now.action = received.action;
	}
}

void PC_send(void) {
	uint8_t tmp[14];
	f3_u812_convert conv;
	conv.f[0] = now.x;
	conv.f[1] = now.y;
	conv.f[2] = now.theta;
	for (int i = 0; i < 12; ++i) {
		tmp[i] = conv.u8[i];
	}
	tmp[12] = now.field;
	tmp[13] = now.action;
	uartPC.write(tmp, 14);
}
#endif

void DD_send(void) {
#ifdef DD_DEBUG
	union {double d[10]; int i[20]; uint8_t u8[80];} tmp;
	for (int i = 0; i < 4; ++i) tmp.d[i] = mtr_now[i];
	for (int i = 0; i < 4; ++i) tmp.d[i+4] = mtr_target[i];
	for (int i = 0; i < 4; ++i) tmp.i[i+16] = mtr_rq[i];
	uartDD.write(tmp.u8,80);
#else
	uint8_t tmp = 0;
	switch (now.action) {
	case 1:
		tmp |= 0x01;
		break;
	case 2:
		tmp |= 0x02;
		tmp |= 0x04;
		break;
	case 3:
		tmp |= 0x02;
		tmp |= 0x04;
		break;
	case 4:
		tmp |= 0x02;
		tmp |= 0x04;
		break;
	}
	uartDD.write(&tmp,1);
#endif
}

double debug[32];
void Sensor_receive(void) {
	union {float f[3]; uint8_t u8[12];} tmp;
	if (uartSensor.read(tmp.u8)) {
		now.x = tmp.f[0];
		now.y = tmp.f[1];
		now.theta = tmp.f[2];
	}
}

#ifdef DISABLE_PC
bool flag = true;
#endif
//run every 1ms
void main_interrupt(void) {
	//communicate
	Sensor_receive();
#ifdef DISABLE_PC
	if (sw.read()) {
		flag = true;
	} else {
		if (flag) {
			received.move_spd += 200;
			if (received.move_spd > 500) received.move_spd = 0;
			flag = false;
		}
	}
	received.move_dir = 0;
	received.rot = 0;
	received.action = 0;
	received.field = 0;
#else
	PC_receive();
	PC_send();
#endif
	DD_send();

	rq_abs.x = static_cast<double>(received.move_spd)*cos(-received.move_dir /180.0*M_PI);
	rq_abs.y = static_cast<double>(received.move_spd)*sin(-received.move_dir /180.0*M_PI);
	rq_theta = received.rot/360.0*BODY_DIAMETER*PI;

	//localize rq
	rq_local = Point::rotated(rq_abs, -now.theta/180*PI);

	//communicate with motor
	mtr_target[0] = (Point::rotated(rq_local, 135).x*2+rq_theta)/PI/TIRE_DIAMETER;
	mtr_target[1] = (Point::rotated(rq_local, 45).x*2+rq_theta)/PI/TIRE_DIAMETER;
	mtr_target[2] = (Point::rotated(rq_local, -45).x*2+rq_theta)/PI/TIRE_DIAMETER;
	mtr_target[3] = (Point::rotated(rq_local, -135).x*2+rq_theta)/PI/TIRE_DIAMETER;
	for (int i = 0; i < 4; i++) {
		mtr_now[i] = culc_ave[i].culc(static_cast<double>(enc[i].read())) / 2048.0 * 1000.0;
		enc[i].reset();
	}
	mtr_now[1] *= -1;
	for (int i = 0; i < 4; i++) {
		if (mtr_target[i] == 0) {
			mtr_pid[i].reset();
			debug[i*3] = 0;
			debug[i*3+1] = 0;
			debug[i*3+2] = 0;
			mtr_rq[i] = 0;
		} else {
			mtr_rq[i] = mtr_pid[i].control(mtr_target[i],mtr_now[i],1);
			debug[i*3] = mtr_pid[i].getControlValue(ControlType::P);
			debug[i*3+1] = mtr_pid[i].getControlValue(ControlType::I);
			debug[i*3+2] = mtr_pid[i].getControlValue(ControlType::D);
			if (mtr_rq[i] > 95) mtr_rq[i] = 95;
			if (mtr_rq[i] < -95) mtr_rq[i] = -95;
		}
		mtr[i].write(mtr_rq[i]);
	}
}

//main
int main(void)
{
	sken_system.init();
	received.move_spd = 0;

	//左のコネクタから順番に
	mtr[0].init(Apin,B8,TIMER10,CH1);
	mtr[0].init(Bpin,B9,TIMER11,CH1);
	mtr[1].init(Apin,A6,TIMER13,CH1);
	mtr[1].init(Bpin,A7,TIMER14,CH1);
	mtr[2].init(Apin,A8,TIMER1,CH1);
	mtr[2].init(Bpin,A11,TIMER1,CH4);
	mtr[3].init(Apin,B14,TIMER12,CH1);
	mtr[3].init(Bpin,B15,TIMER12,CH2);

	enc[0].init(A0,A1,TIMER5,TIRE_DIAMETER,2048);
	enc[1].init(B3,A5,TIMER2,TIRE_DIAMETER,2048);
	enc[2].init(B6,B7,TIMER4,TIRE_DIAMETER,2048);
	enc[3].init(C6,C7,TIMER3,TIRE_DIAMETER,2048);


	mtr_pid[0].setGain(40,20,15,20);
	mtr_pid[1].setGain(40,20,15,20);
	mtr_pid[2].setGain(40,20,15,20);
	mtr_pid[3].setGain(40,20,15,20);

	uartDD.init(USB_B,115200);
	uartSensor.init(USB_A,115200);
#ifdef DISABLE_PC
	sw.init(C13,INPUT_PULLUP);
#else
	uartPC.init(USB_miniB,9600);
#endif

	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {
	}
}
