/*
 * define_data.h
 *
 *  Created on: 2023/05/03
 *      Author: p1ing
 */

#ifndef DEFINE_DATA_H_
#define DEFINE_DATA_H_

enum Asimode{
	omuni3,
	omuni4,
	mekanum,
	dokusute,
};
enum Place{
	FR,
	FL,
	BR,
	BL,
	FR_Dokusute,
	FL_Dokusute,
	BR_Dokusute,
	BL_Dokusute,
};

struct DebugData {
	double enc[8],out[8],target[8];
};

#endif /* DEFINE_DATA_H_ */
