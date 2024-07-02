/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa404
  * @version V0.3.5
  * @date    02-July-2024
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
struct Average {
private:
	double BUFFER_[10] = {};
public:
	Average(void) = default;
	double culc(double x) {
		long double tmp = 0;
		for (int i = 0; i < 9; ++i) {
			BUFFER_[i] = BUFFER_[i+1];
			tmp += BUFFER_[i];
		}
		BUFFER_[9] = x;
		return (tmp+x)/10;
	}
} culc_ave[4];
union f3_u812_convert {
	float f[3];
	uint8_t u8[12];
};

Point rq_abs; double rq_theta; //絶対座標系での指定{{x[mm/s],y[mm/s]},theta[deg]}
Point rq_local;
double mtr_target[4]; //モーター目標速度[mm/s]
Pid mtr_pid[4];
double mtr_rq[4]; //モーター出力[% <95]
Motor mtr[4];
Encoder enc[4]; Encoder_data enc_data[4];
double mtr_now[4]; //[mm/s]

MyUart<0> uartDD;
MyUart<12> uartSensor;
MyUart<14> uartPC;

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

void DD_send(void) {
	/*
	union {double d[10]; int i[20]; uint8_t u8[80];} tmp;
	for (int i = 0; i < 4; ++i) tmp.d[i] = mtr_now[i];
	for (int i = 0; i < 4; ++i) tmp.d[i+4] = mtr_target[i];
	for (int i = 0; i < 4; ++i) tmp.i[i+16] = mtr_rq[i];
	uartDD.write(tmp.u8,80);
	*/
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
}

uint8_t debug[32];
void Sensor_receive(void) {
	union {float f[3]; uint8_t u8[12];} tmp;
	uartSensor.get_raw(debug);
	if (uartSensor.read(tmp.u8)) {
		now.x = tmp.f[0];
		now.y = tmp.f[1];
		now.theta = tmp.f[2];
	}
}

//run every 1ms
void main_interrupt(void) {
	//communicate
	Sensor_receive();
	PC_receive();
	DD_send();
	PC_send();

	//read encoder
	for(int i = 0; i < 4; ++i) enc[i].interrupt(&enc_data[i]);
	enc_data[1].volcity *= -1;
	for(int i = 0; i < 4; ++i) mtr_now[i] = culc_ave[i].culc(enc_data[i].volcity);

	if ((received.move_spd==0 && received.move_dir==0 && received.rot==0) || received.action==255) {
		for (int i = 0; i < 4; ++i) {
			mtr_rq[i] = 0;
			mtr_pid[i].reset();
			mtr_target[i] = 0;
		}
	} else {
		rq_abs.x = static_cast<double>(received.move_spd)*cos(-received.move_dir /180.0*M_PI);
		rq_abs.y = static_cast<double>(received.move_spd)*sin(-received.move_dir /180.0*M_PI);
		rq_theta = received.rot/360.0*BODY_DIAMETER*PI;

		//localize rq
		rq_local = Point::rotated(rq_abs, -now.theta/180*PI);

		//communicate with motor
		mtr_target[0] = Point::rotated(rq_local, 135).x*2-rq_theta;
		mtr_target[1] = Point::rotated(rq_local, 45).x*2-rq_theta;
		mtr_target[2] = Point::rotated(rq_local, -45).x*2-rq_theta;
		mtr_target[3] = Point::rotated(rq_local, -135).x*2-rq_theta;
		//mtr_write
		for (int i = 0; i < 4; ++i) {
			mtr_rq[i] = mtr_pid[i].control(mtr_target[i],mtr_now[i],1);
		}
	}
	for(int i = 0; i < 4; ++i) {
		if (mtr_rq[i] > 95) mtr_rq[i] = 95;
		if (mtr_rq[i] < -95) mtr_rq[i] = -95;
		mtr[i].write(static_cast<int>(mtr_rq[i]));
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

	for (int i = 0; i < 4; ++i) mtr_pid[i].setGain(0.1,0,0,20);

	uartDD.init(USB_B,115200);
	uartSensor.init(USB_A,115200);
	uartPC.init(USB_miniB,9600);

	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {
	}
}
