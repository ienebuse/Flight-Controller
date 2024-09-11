/*
 * \file MadgwickFusion.h
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
#ifndef MADGWICKFUSION_H_
#define MADGWICKFUSION_H_

#include "madgwick/Fusion.h"
#include <Filter.h>

class MadgwickFusion : public Filter
{
public:
    MadgwickFusion(Attitude* att);
    virtual ~MadgwickFusion();

    bool init();
    void reset();
//    virtual void update(Vector3f gyro, Vector3f acc, Vector3f mag, bool magAvailable = false);
    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dT, bool magAvailable = true);
    void getEuler();

private:
    FusionAhrs m_ahrs;
};

#endif /* MADGWICKFUSION_H_ */
