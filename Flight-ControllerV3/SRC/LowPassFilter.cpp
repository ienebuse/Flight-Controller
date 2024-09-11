/*
 * LowPassFilter.cpp
 *
 *  Created on: Aug 14, 2024
 *      Author: IEnebuse
 */

#include <LowPassFilter.h>
#include <cmath>

//#define SQRT_2	1.4142135623730951

LowPassFilter::LowPassFilter() {
	// TODO Auto-generated constructor stub

}

LowPassFilter::~LowPassFilter() {
	// TODO Auto-generated destructor stub
}

void LowPassFilter::init(float cutoffFreq, float samplingFreq) {
	m_cutoffFreq = cutoffFreq;
	m_samplingRate = samplingFreq;
	m_normalizedCutoff = m_cutoffFreq/m_samplingRate;


	float ita = 1/tan(M_PI * m_normalizedCutoff);
	static const float SQRT_2 = sqrt(2);

	b0 = 1/(1+ SQRT_2*ita + ita*ita);
	b1 = 2 * b0;
	b2 = b0;

	a0 = 1;
	a1 = 2 * (1 - ita*ita) * b0;
	a2 = (1 - SQRT_2*ita + ita*ita) * b0;
}

float LowPassFilter::apply(float input) {
	static float last_input = 0;
	static float alpha = 0.98;
	u = input;
	float output = (b0 * u + b1 * u_1 + b2 * u_2 - a1 * y_1 - a2 * y_2);
	y_2 = y_1;
	y_1 = output;
	u_2 = u_1;
	u_1 = u;

//	float output = alpha * input + (1-alpha)*last_input;
//	last_input = output;

	return output;
}

