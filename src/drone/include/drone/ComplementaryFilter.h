#if !defined( COMPLEMENTARY_FILTER_H )
#define COMPLEMENTARY_FILTER_H


#include <math.h>


class ComplementaryFilter
{
    public:
        
        float alpha;
        float Qam[4];
        float Qw[4];
        float q[4];
        float b[3];
        float pitch, roll, yaw;
        ComplementaryFilter(float alpha);

        void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, double dt);
        
        float getPitch()
        {
                // return atan2f(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]) * 57.29578f;
                return atan2(2.0*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2])) * 57.29578f;
        }

        float getRoll()
        {
            // return asinf(-2.0f * (q[1]*q[3] - q[0]*q[2])) * 57.29578f;
            return asin(2.0f * (q[0]*q[2] - q[1]*q[3])) * 57.29578f;
        }

        float getYaw()
        {
            // return atan2f(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]) * 57.29578f;
            return atan2(2.0*(q[1]*q[2] + q[0]*q[3]), 1 - 2*(q[2]*q[2] + q[3]*q[3])) * 57.29578f;
        }

        float invSqrt(float x);
};


#endif