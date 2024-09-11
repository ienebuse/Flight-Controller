/*
 * \file MadgwickFusion.cpp
 *
 *  \author arothwel
 *  \brief 
 */
//-------------------------------------------------------------------------//
// Copyright (c) Raymarine UK Limited 2023
//
// Reproduction or transmission in whole or in part (whether by photocopying or
// storing in any medium by electronic means or otherwise) without the written
// permission of Raymarine UK Limited is prohibited.
//
// Confidential
//-------------------------------------------------------------------------//
#include <MadgwickFusion.h>

MadgwickFusion::MadgwickFusion(Attitude* att) : Filter(att)
{
    // TODO Auto-generated constructor stub

}

MadgwickFusion::~MadgwickFusion()
{
    // TODO Auto-generated destructor stub
}

bool MadgwickFusion::init()
{

    // Gain 0.7

//    double rejectionTimeout = 10.0 / ((float)FLIGHT_CONTROL_PERIOD_US/1000000);

    //FusionAhrsInitialise(&m_ahrs);
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.7f,
            .accelerationRejection = 90.0f, // 10 seems to be used on examples but we may need to tweak this for high shock rejection
            .magneticRejection = 20.0f,
            .rejectionTimeout = (unsigned int)(10/0.0021f),//(unsigned int)rejectionTimeout,
    };
    FusionAhrsSetSettings(&m_ahrs, &settings);
    FusionAhrsReset(&m_ahrs);

    return true;
}

void MadgwickFusion::reset()
{
    FusionAhrsReset(&m_ahrs);
}

void MadgwickFusion::update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable) {
    FusionVector _gyro;
    FusionVector _acc;
    FusionVector _mag;

    dT = 0.0021f;

//    float mSqInv = 1/sqrt(mx*mx + my*my + mz*mz);
//	mx *= mSqInv;
//	my *= mSqInv;
//	mz *= mSqInv;

    _gyro.array[0] = gx;
    _gyro.array[1] = gy;
    _gyro.array[2] = gz;

    _acc.array[0] = ax;
    _acc.array[1] = ay;
    _acc.array[2] = az;

    _mag.array[0] = mx;
    _mag.array[1] = my;
    _mag.array[2] = mz;

    if(magAvailable) {
    	FusionAhrsUpdate(&m_ahrs, _gyro, _acc, _mag, dT);
    }
	else {
		FusionAhrsUpdateNoMagnetometer(&m_ahrs, _gyro, _acc, dT);
	}
    att->quat.q0 = m_ahrs.quaternion.element.w;
    att->quat.q1 = m_ahrs.quaternion.element.x;
    att->quat.q2 = m_ahrs.quaternion.element.y;
    att->quat.q3 = m_ahrs.quaternion.element.z;
    getEuler();
}

//bool MadgwickFusion::update(Vector3f gyro, Vector3f acc, Vector3f mag, bool magAvailable)
//{
//    FusionVector _gyro;
//    FusionVector _acc;
//    FusionVector _mag;
//
//    _gyro.array[0] = gyro.v[0];
//    _gyro.array[1] = gyro.v[1];
//    _gyro.array[2] = gyro.v[2];
//
//    _acc.array[0] = acc.v[0];
//    _acc.array[1] = acc.v[1];
//    _acc.array[2] = acc.v[2];
//
//    _mag.array[0] = mag.v[0];
//    _mag.array[1] = mag.v[1];
//    _mag.array[2] = mag.v[2];
//
//#ifdef DEVBOARD
//    FusionAhrsUpdate(&m_ahrs, _gyro, _acc, _mag,m_sampleTime);
//#else
//    FusionAhrsUpdateNoMagnetometer(&m_ahrs, _gyro, _acc,m_sampleTime);
//#endif
//
//    return true;
//}

void MadgwickFusion::getEuler()
{
    FusionEuler feuler;

    feuler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&m_ahrs));

    att->euler.r = feuler.angle.roll;
    att->euler.p = feuler.angle.pitch;
    att->euler.y = feuler.angle.yaw;
}



