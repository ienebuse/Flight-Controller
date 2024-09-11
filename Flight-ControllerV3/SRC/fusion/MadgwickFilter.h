/*
 * MadgwickFilter.h
 *
 *  Created on: Jan 19, 2023
 *      Author: enebusei
 */

#ifndef MADGWICKFILTER_H_
#define MADGWICKFILTER_H_


#include "Filter.h"


class Madgwick : public Filter {

private:

	const float beta{0.001f * M_PI * 180}, m_alpha{0.98};
	Attitude compAtt;

public:

	Madgwick(Attitude* att);

//	void init(Filter_setup);

	void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable = true);
	void update(float ax, float ay, float az,float gx, float gy, float gz, float dT);

	void compUpdate(float ax, float ay, float az,float gx, float gy, float gz, float dT);
};

#endif /* MADGWICKFILTER_H_ */
