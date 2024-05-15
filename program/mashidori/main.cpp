/**
  ******************************************************************************
  * @file    main.cpp
  * @author  Tikuwa
  * @version V0.1.1
  * @date    15-May-2024
  * @brief   R_B main function.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "sken_library/include.h"

#include <cmath>

using std::sin;
using std::cos;

constexpr double SPD = 2.0;
constexpr double DIAMETER = 75.0; //[mm]
constexpr double TEST = 244;

//ロボマスモータのエンコーダ受信用
class ROBOMAS_Encoder_Data {
public:
	ROBOMAS_Encoder_Data() :sto({0,0,0,0}) {}
	short operator[](int i) {
		return (sto.uint8_data[i*2]<<8)+sto.uint8_data[i*2+1];
	}
	void update(uint8_t *data) {
		for (int i = 0; i < 8; ++i) {
			sto.uint8_data[i] = data[i];
		}
	}
private:
	union {
		short short_data[4];
		uint8_t uint8_data[8];
	} sto;
};
class Point {
public:
	double x;
	double y;
	Point(double i=0, double j=0) : x(i),y(j) {}
	//rotate point
	//in = {x, y, theta_度数法}
	void rotate(double theta) {
		double tmpx = x, tmpy = y;
		double rad = theta / 180 * M_PI;
		x = tmpx * cos(rad) - tmpy * sin(rad);
		y = tmpx * sin(rad) + tmpy * cos(rad);
	}
	Point operator=(Point i) {
		this->x = i.x;
		this->y = i.y;
		return *this;
	}
};

CanData can_received;
uint8_t send_data[8];
Point abs_rq_p; double rq_theta; //絶対座標系での指定{{x,y},theta}
Point local_rq_p, mtr_rq_p;
double direction = 0; //置く向きに合わせて初期値を変える。(0~360)
ROBOMAS_Encoder_Data enc_data[3];
short debug1[4],debug2[4],debug3[4];
Gpio sw;

void can_interrupt(CanRxMsgTypeDef);
void stick_resize(double&,double&);
void get_rq(void);
void main_interrupt(void);

//run by can_handler
void can_interrupt(CanRxMsgTypeDef rx) {
	if (can_received.rx_stdid == 0x201) {
		enc_data[0].update(can_received.rx_data);
	}
	if (can_received.rx_stdid == 0x202) {
		enc_data[1].update(can_received.rx_data);
	}
	if (can_received.rx_stdid == 0x203) {
		enc_data[2].update(can_received.rx_data);
	}
}

//return -1~1
void stick_resize(double &x, double &y) {
	double longer = (abs(x) > abs(y) ? abs(x) : abs(y));
	double angle = std::atan2(y, x);

	x = cos(angle) * longer / 64;
	y = sin(angle) * longer / 64;
}

/*
PS3 ps3;
PS3_data ps3_data;
*/
//communicate
void get_rq(void) {
	double rqx,rqy;
	/*   unavailable    */
	/*   coming soon    */
	/* work in progress */
	/*
	ps3.Getdata(&ps3_data);
	rqx = ps3_data.LxPad;
	rqy = ps3_data.LyPad;
	stick_resize(rqx, rqy);
	*/
	rqx = (sw.read()?0:1);
	rqy = (sw.read()?1:0);
	rq_theta = 0;

	//test
	abs_rq_p.x = rqx*SPD;
	abs_rq_p.y = rqy*SPD;
}

double debugd, sum = 0;
//run every 1ms
//モーターは正の出力で反時計回り
void main_interrupt(void) {
	static union{short s; uint8_t u[2];} conv = {0};

	//communicate
	get_rq();

	//localize rq
	local_rq_p = abs_rq_p;
	local_rq_p.rotate(-direction);

	for (int i = 0; i < 4; ++i) debug1[i] = enc_data[0][i];
	for (int i = 0; i < 4; ++i) debug2[i] = enc_data[1][i];
	for (int i = 0; i < 4; ++i) debug3[i] = enc_data[2][i];
	debugd = static_cast<double>(debug1[1])/36.0/60.0*TEST;
	sum += debugd/1000.0;

	//communicate with motor
	//front
	mtr_rq_p = local_rq_p;
	conv.s = static_cast<short>(mtr_rq_p.x + rq_theta*SPD);
	send_data[0] = conv.u[0];
	send_data[1] = conv.u[1];
	//left
	mtr_rq_p = local_rq_p;
	mtr_rq_p.rotate(-120);
	conv.s = static_cast<short>(mtr_rq_p.x + rq_theta*SPD);
	send_data[2] = conv.u[0];
	send_data[3] = conv.u[1];
	//right
	mtr_rq_p = local_rq_p;
	mtr_rq_p.rotate(-240);
	conv.s = static_cast<short>(mtr_rq_p.x + rq_theta*SPD);
	send_data[4] = conv.u[0];
	send_data[5] = conv.u[1];
	//none
	send_data[6] = 0;
	send_data[7] = 0;
	static int pres = 0;
	if (++pres == 10) {
		sken_system.canTransmit(CAN_2,0x200,send_data,8);
		pres = 0;
	}
}

//main
int main(void)
{
	sken_system.init();
	sw.init(C13,INPUT_PULLUP);

	sken_system.startCanCommunicate(B13,B12,CAN_2);
	sken_system.addCanRceiveInterruptFunc(CAN_2,&can_received,can_interrupt);

	//ps3.StartRecive();
	sken_system.addTimerInterruptFunc(main_interrupt,0,1);

	while (true) {}
}
