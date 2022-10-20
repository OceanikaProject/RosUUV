#if !defined( MADGWICK_FILTER_H )
#define MADGWICK_FILTER_H

#include <math.h>


class MadgwickFilter
{
  public:
    MadgwickFilter();
    void begin(float sampleFreq, float betaDef) 
    { 
        invSampleFreq = 1.0f / sampleFreq;
        beta = betaDef;
    }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float invSqrt(float x);
    void computeAngles();
    char anglesComputed;
    float invSampleFreq, beta;
    float q0, q1, q2, q3;
    float roll, pitch, yaw;
    float *q_mult(float q[], float r[]);
    float *q_rotate(float point[], float q[]);
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
};

#endif