/*
 * VqfFilter.h
 *
 *  Created on: Jun 28, 2024
 *      Author: Ikenna
 */

#ifndef INC_VQFFILTER_H_
#define INC_VQFFILTER_H_


#include <fusion/Filter.h>
#include <fusion/vqf.hpp>

class VqfFilter : Filter {
public:
	VqfFilter(Attitude* att);
	virtual ~VqfFilter();

//	void init(Filter_setup);

	void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable = true);

	void update(float ax, float ay, float az, float gx, float gy, float gz, float dT);

private:
	VQF m_vqf;

};

#endif /* INC_VQFFILTER_H_ */
