/*
 * asimawari.h
 *
 *  Created on: 2023/04/27
 *      Author: p1ing
 */

#ifndef ASIMAWARI_H_
#define ASIMAWARI_H_

#include "sken_library/include.h"
#include "define_data.h"

class Asimawari {
public:
	void turn(Asimode asimode,double lx,double ly,double rx,double robot_diameter,double wheel_radius);
	void mtr_pin_init(Place mtr_Pin,MtrPin mtr_pin,Pin pin,TimerNumber timer,TimerChannel ch);
	void enc_pin_init(Place enc_Pin,Pin pin1,Pin pin2,TimerNumber timer,int diameter);
	void pid_set(Place pid_place,double p,double i,double d);
	DebugData get_debug_data();
private:
	void Omuni_3();
	void Omuni_4();
	void Mekanumu_M();
	void Dokusute_D();
	Motor mtr[8];
	Encoder encoder[8];
	Encoder_data data[8];
	Pid pid_control[8];
	DebugData debugdata;
	double Lx,Ly,Rx,DI,WH;
};




#endif /* ASIMAWARI_H_ */
