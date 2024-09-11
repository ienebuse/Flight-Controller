/*
 * ComplimentaryFilter.h
 *
 *  Created on: Jun 28, 2024
 *      Author: Ikenna
 */

#ifndef INC_COMPLIMENTARYFILTER_H_
#define INC_COMPLIMENTARYFILTER_H_

#include <fusion/Filter.h>

class ComplimentaryFilter: public Filter {
public:
	ComplimentaryFilter(Attitude* att);
	virtual ~ComplimentaryFilter();

//	void init(Filter_setup);

	void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable = true);

private:
	float m_alpha{0.98};
};

#endif /* INC_COMPLIMENTARYFILTER_H_ */
