#include <drone/ComplementaryFilter.h>

#include <ros/console.h>

ComplementaryFilter::ComplementaryFilter(float alpha)
{
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    this->alpha = alpha;
}

void ComplementaryFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, double dt)
{
    float recipNorm;
    float accel_length = sqrt(ax*ax + ay*ay + az*az);
    float magn_length = sqrt(mx*mx + my*my + mz*mz);

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Normalise accelerometer measurement
    // recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax /= accel_length;
    ay /= accel_length;
    az /= accel_length;

    // cout << ax << " | " << ay << " | " << az << endl;
    // ROS_INFO("%f | %f | %f", ax ,ay, az);

    // // Normalise magnetometer measurement
    // recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx /= magn_length;
    my /= magn_length;
    mz /= magn_length;

    Qw[0] = q[0] - gx*q[1]*dt/2 - q[2]*gy*dt/2 - q[3]*gz*dt/2;
    Qw[1] = q[1] + gx*q[0]*dt/2 - q[3]*gy*dt/2 + q[2]*gz*dt/2;
    Qw[2] = q[2] + gx*q[3]*dt/2 + q[0]*gy*dt/2 - q[1]*gz*dt/2;
    Qw[3] = q[3] - gx*q[2]*dt/2 + q[1]*gy*dt/2 + q[0]*gz*dt/2;

    roll = atan2(ay, az);
    pitch = atan2(-ax, sqrt(ay*ay + az*az));

    b[0] = mx*cos(roll) + my*sin(roll)*sin(pitch) + mz*sin(roll)*cos(pitch);
    b[1] = my*cos(pitch) - mz*sin(pitch);
    b[2] = -mx*sin(pitch) + my*cos(roll)*sin(pitch) + mz*cos(roll)*cos(pitch);

    yaw = atan2(-b[1], b[0]);

    Qam[0] = cos(pitch/2)*cos(roll/2)*cos(yaw/2) + sin(pitch/2)*sin(roll/2)*sin(yaw/2);
    Qam[1] = sin(pitch/2)*cos(roll/2)*cos(yaw/2) - cos(pitch/2)*sin(roll/2)*sin(yaw/2);
    Qam[2] = cos(pitch/2)*sin(roll/2)*cos(yaw/2) + sin(pitch/2)*cos(roll/2)*sin(yaw/2);
    Qam[3] = cos(pitch/2)*cos(roll/2)*sin(yaw/2) - sin(pitch/2)*sin(roll/2)*cos(yaw/2);

    q[0] = (1 - alpha) * Qw[0] + alpha * Qam[0];
    q[1] = (1 - alpha) * Qw[1] + alpha * Qam[1];
    q[2] = (1 - alpha) * Qw[2] + alpha * Qam[2];
    q[3] = (1 - alpha) * Qw[3] + alpha * Qam[3];
}

float ComplementaryFilter::invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}