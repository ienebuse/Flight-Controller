/*
 * MahonyFilter.h
 *
 *  Created on: Jan 19, 2023
 *      Author: enebusei
 */

#ifndef INC_MAHONYFILTER_H_
#define INC_MAHONYFILTER_H_


#include "Filter.h"


class Mahony : public Filter {
private:

	float Kp;
	float Ki;
	float eInt[3] = {0.0f, 0.0f, 0.0f};

public:

	Mahony(Attitude* att);

	void init(Filter_setup);

	void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable = true);
};


#endif /* INC_MAHONYFILTER_H_ */
