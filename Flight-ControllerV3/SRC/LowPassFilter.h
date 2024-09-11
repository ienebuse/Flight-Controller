/*
 * LowPassFilter.h
 *
 *  Created on: Aug 14, 2024
 *      Author: IEnebuse
 */

#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

class LowPassFilter {
public:
	LowPassFilter();
	virtual ~LowPassFilter();

	void init(float cutoffFreq, float samplingFreq);

	float apply(float input);

private:
// Butterworth filter parameters
	float m_cutoffFreq;	//Set the cutoff frequency (as a fraction of the Nyquist frequency)
	float m_samplingRate;	//Calculate sampling rate based on time steps
	float m_normalizedCutoff;

	float a0, a1, a2, b0, b1, b2;

	float y=0, y_1=0, y_2=0;
	float u=0, u_1=0, u_2=0;
};

#endif /* LOWPASSFILTER_H_ */
