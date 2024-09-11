/*
 * Filter.h
 *
 *  Created on: Jan 19, 2023
 *      Author: enebusei
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#include <typedefs.h>

//typedef struct
//{
//	float r = 0;
//	float p = 0;
//	float y = 0;
//}EulerAttitude;
//
//typedef struct
//{
//	float q0 = 1;
//	float q1 = 0;
//	float q2 = 0;
//	float q3 = 0;
//}QuatAttitude;


class Filter {

protected:
	Attitude* att;
	void* mp_setupValue;
	Vector_t<float> magRef;


public:

	Filter(Attitude* att, void* setupValue=nullptr);

	virtual void init(Vector_t<float> mag) {
		magRef.x = mag.x;
		magRef.y = mag.y;
		magRef.z = mag.z;
	}

//	Filter(EulerAttitude* e);

	virtual ~Filter();

//	virtual void init(Filter_setup) = 0;

	virtual void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable = true) = 0;

	virtual void getInitialOrientation(float ax, float ay, float az, float mx, float my, float mz);

	virtual void reset() {

	}

};


#endif /* INC_FILTER_H_ */
