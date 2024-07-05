/*
 * asimawari.cpp
 *
 *  Created on: 2023/04/27
 *      Author: p1ing
 */

#include "asimawari.h"
#include "math.h"
#include "string.h"
#include "define_data.h"

void Asimawari::mtr_pin_init(Place mtr_place,MtrPin mtr_pin,Pin pin,TimerNumber timer,TimerChannel ch) {
	switch(mtr_place)
	{
	case FR:mtr[0].init(mtr_pin,pin,timer,ch);break;
	case FL:mtr[1].init(mtr_pin,pin,timer,ch);break;
	case BR:mtr[2].init(mtr_pin,pin,timer,ch);break;
	case BL:mtr[3].init(mtr_pin,pin,timer,ch);break;
	case FR_Dokusute:mtr[4].init(mtr_pin,pin,timer,ch);break;
	case FL_Dokusute:mtr[5].init(mtr_pin,pin,timer,ch);break;
	case BR_Dokusute:mtr[6].init(mtr_pin,pin,timer,ch);break;
	case BL_Dokusute:mtr[7].init(mtr_pin,pin,timer,ch);break;
	}
}

void Asimawari::enc_pin_init(Place enc_place,Pin pin1,Pin pin2,TimerNumber timer,int wheel_diameter){
	switch(enc_place){
	case FR:encoder[0].init(pin1,pin2,timer,wheel_diameter);break;
	case FL:encoder[1].init(pin1,pin2,timer,wheel_diameter);break;
	case BR:encoder[2].init(pin1,pin2,timer,wheel_diameter);break;
	case BL:encoder[3].init(pin1,pin2,timer,wheel_diameter);break;
	case FR_Dokusute:encoder[4].init(pin1,pin2,timer,wheel_diameter);break;
	case FL_Dokusute:encoder[5].init(pin1,pin2,timer,wheel_diameter);break;
	case BR_Dokusute:encoder[6].init(pin1,pin2,timer,wheel_diameter);break;
	case BL_Dokusute:encoder[7].init(pin1,pin2,timer,wheel_diameter);break;
	}
}

void Asimawari::pid_set(Place pid_place,double p,double i,double d){
	switch(pid_place){
	case FR: pid_control[0].setGain(p,i,d); break;
	case FL: pid_control[1].setGain(p,i,d); break;
	case BR: pid_control[2].setGain(p,i,d); break;
	case BL: pid_control[3].setGain(p,i,d); break;
	case FR_Dokusute: pid_control[4].setGain(p,i,d); break;
	case FL_Dokusute: pid_control[5].setGain(p,i,d); break;
	case BR_Dokusute: pid_control[6].setGain(p,i,d); break;
	case BL_Dokusute: pid_control[7].setGain(p,i,d); break;
	}
}

void Asimawari::turn(Asimode asimode,double lx,double ly,double rx,double robot_diameter,double wheel_radius){
	Lx = lx; Ly = ly; Rx = rx * M_PI /180;
	DI = robot_diameter; WH = wheel_radius;
	switch(asimode) {

	case omuni3:
		debugdata.target[0] = (-Lx + DI * Rx) / (2 * M_PI * WH);
		debugdata.target[1] = (Lx / 2 - Ly * sqrt(3) / 2 + DI * Rx) / (2 * M_PI * WH);
		debugdata.target[2] = (Lx / 2 + Ly * sqrt(3) / 2 + DI * Rx) / (2 * M_PI * WH);
		for(int i=0;i<3;i++)
		{
			debugdata.enc[i] = data[i].rps;
		}
	break;

	case omuni4:
		debugdata.target[0] = (-(sqrt(2)/2)*Lx + (sqrt(2)/2)*Ly + DI*Rx) / (2 * M_PI * WH);
		debugdata.target[1] = ((sqrt(2)/2)*Lx + (sqrt(2)/2)*Ly + DI*Rx) / (2 * M_PI * WH);
		debugdata.target[2] = ((sqrt(2)/2)*Lx + -(sqrt(2)/2)*Ly + DI*Rx) / (2 * M_PI * WH);
		debugdata.target[3] = (-(sqrt(2)/2)*Lx + -(sqrt(2)/2)*Ly + DI*Rx) / (2 * M_PI * WH);
		debugdata.target[4] = (-Lx + DI * Rx) / (2 * M_PI * WH);
		debugdata.target[5] = (-Ly + DI * Rx) / (2 * M_PI * WH);
		debugdata.target[6] = (Lx + DI * Rx) / (2 * M_PI * WH);
		debugdata.target[7] = (Ly + DI * Rx) / (2 * M_PI * WH);
		for(int i=0;i<4;i++)
		{
			debugdata.enc[i] = -data[i].rps;
		}
	break;

	case mekanum:
		debugdata.target[2] = ((-Lx + Ly) / sqrt(2) + DI * Rx) / (2 * M_PI * WH);
		debugdata.target[1] = ((-Lx - Ly) / sqrt(2) + DI * Rx) / (2 * M_PI * WH);
		debugdata.target[0] = ((Lx - Ly) / sqrt(2) + DI * Rx) / (2 * M_PI * WH);
		debugdata.target[3] = ((Lx + Ly) / sqrt(2) + DI * Rx) / (2 * M_PI * WH);
		for(int i=0;i<4;i++)
		{
			debugdata.enc[i] = data[i].rps;
		}
	break;

	case dokusute:
		debugdata.target[0] = sqrt(pow((Lx - DI * Rx*sqrt(2)),2)+pow((Ly + DI * Rx*sqrt(2)),2)) / (2 * M_PI * WH);
		debugdata.target[1] = sqrt(pow((Lx - DI * Rx*sqrt(2)),2)+pow((Ly - DI * Rx*sqrt(2)),2)) / (2 * M_PI * WH);
		debugdata.target[2] = sqrt(pow((Lx + DI * Rx*sqrt(2)),2)+pow((Ly - DI * Rx*sqrt(2)),2)) / (2 * M_PI * WH);
		debugdata.target[3] = sqrt(pow((Lx + DI * Rx*sqrt(2)),2)+pow((Ly + DI * Rx*sqrt(2)),2)) / (2 * M_PI * WH);

		debugdata.target[4] = (atan2((Ly + DI * Rx*sqrt(2)),(Lx - DI * Rx*sqrt(2))))/M_PI*180;
		debugdata.target[5] = (atan2((Ly - DI * Rx*sqrt(2)),(Lx - DI * Rx*sqrt(2))))/M_PI*180;
		debugdata.target[6] = (atan2((Ly - DI * Rx*sqrt(2)),(Lx + DI * Rx*sqrt(2))))/M_PI*180;
		debugdata.target[7] = (atan2((Ly + DI * Rx*sqrt(2)),(Lx + DI * Rx*sqrt(2))))/M_PI*180;
		for(int s=0;s<4;s++)
		{
			debugdata.enc[s] = data[s].rps;
			debugdata.enc[s+4] = data[s+4].deg;
		}
	break;
	}

	if(Lx == 0 && Ly == 0 && Rx == 0)for(int r=0;r<4;r++)pid_control[r].reset();

	for(int a=0;a<8;a++){
		mtr[a].write(debugdata.out[a]);
		encoder[a].interrupt(&data[a]);
		debugdata.out[a] = pid_control[a].control(debugdata.target[a],debugdata.enc[a],1);
	}
}

DebugData Asimawari::get_debug_data(){
	return debugdata;
}
