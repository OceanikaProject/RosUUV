import numpy as np
from numpy.linalg import norm


class MadgwickAHRS:
    beta = 1.7
    invSampleFreq = 1. / 80.

    def __init__(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def update(self, gyroscope, accelerometer, magnetometer):

        if norm(magnetometer) == 0:
            angles =  self.updateIMU(accelerometer, gyroscope)
            return angles

        gx, gy, gz = gyroscope
        gx = 0.01745329252  * gx
        gy = 0.01745329252  * gy
        gz = 0.01745329252  * gz

        qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        if norm(accelerometer) != 0:
            # Normalise accelerometer measurement
            acc = accelerometer
            # print(accelerometer)
            accelerometer /= norm(accelerometer)
            # print(accelerometer)
            
            # Normalise magnetometer measurement
            magnetometer /= norm(magnetometer)

            mx, my, mz = magnetometer
            ax, ay, az = accelerometer

            # Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0 * self.q0 * mx
            _2q0my = 2.0 * self.q0 * my
            _2q0mz = 2.0 * self.q0 * mz
            _2q1mx = 2.0 * self.q1 * mx
            _2q0 = 2.0 * self.q0
            _2q1 = 2.0 * self.q1
            _2q2 = 2.0 * self.q2
            _2q3 = 2.0 * self.q3
            _2q0q2 = 2.0 * self.q0 * self.q2
            _2q2q3 = 2.0 * self.q2 * self.q3
            q0q0 = self.q0 * self.q0
            q0q1 = self.q0 * self.q1
            q0q2 = self.q0 * self.q2
            q0q3 = self.q0 * self.q3
            q1q1 = self.q1 * self.q1
            q1q2 = self.q1 * self.q2
            q1q3 = self.q1 * self.q3
            q2q2 = self.q2 * self.q2
            q2q3 = self.q2 * self.q3
            q3q3 = self.q3 * self.q3

            # Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * self.q3 + _2q0mz * self.q2 + mx * q1q1 + _2q1 * my * self.q2 + _2q1 * mz * self.q3 - mx * q2q2 - mx * q3q3
            hy = _2q0mx * self.q3 + my * q0q0 - _2q0mz * self.q1 + _2q1mx * self.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * self.q3 - my * q3q3
            _2bx = np.power(hx * hx + hy * hy, .5)
            _2bz = -_2q0mx * self.q2 + _2q0my * self.q1 + mz * q0q0 + _2q1mx * self.q3 - mz * q1q1 + _2q2 * my * self.q3 - mz * q2q2 + mz * q3q3
            _4bx = 2.0 * _2bx
            _4bz = 2.0 * _2bz

            # Gradient decent algorithm corrective step
            s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * self.q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q3 + _2bz * self.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
            s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * self.q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q2 + _2bz * self.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q3 - _4bz * self.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
            s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * self.q2 - _2bz * self.q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q1 + _2bz * self.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q0 - _4bz * self.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
            s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * self.q3 + _2bz * self.q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q0 + _2bz * self.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

            sarr = [s0, s1, s2, s3]
            # print(sarr)
            snorm = norm(sarr)
            sarr /= snorm
            s0, s1, s2, s3 = sarr
            # print(s0, s1, s2, s3)

            # Apply feedback step
            qDot1 -= self.beta * s0
            qDot2 -= self.beta * s1
            qDot3 -= self.beta * s2
            qDot4 -= self.beta * s3

        
        self.q0 = self.q0 + qDot1 * self.invSampleFreq
        self.q1 = self.q1 + qDot2 * self.invSampleFreq
        self.q2 += qDot3 * self.invSampleFreq
        self.q3 += qDot4 * self.invSampleFreq

        # print(self.q0, self.q1, self.q2, self.q3)


        qarr = [self.q0, self.q1, self.q2, self.q3]
        qnorm = norm(qarr)
        qarr /= qnorm
        self.q0, self.q1, self.q2, self.q3 = qarr

        def quaternion_mult(q, r):
            return [
                q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3],
                q[1]*r[0] + q[0]*r[1] - q[3]*r[2] + q[2]*r[3],
                q[2]*r[0] + q[3]*r[1] + q[0]*r[2] - q[1]*r[3],
                q[3]*r[0] - q[2]*r[1] + q[1]*r[2] + q[0]*r[3]
            ]

        def H(q, r):
            return [
                q[0]*r[0] - q[1]*r[1] - q[2]*r[2] - q[3]*r[3],
                q[0]*r[1] + q[1]*r[0] + q[2]*r[3] - q[3]*r[2],
                q[0]*r[2] - q[1]*r[3] + q[2]*r[0] + q[3]*r[1],
                q[0]*r[3] + q[1]*r[2] - q[2]*r[1] + q[3]*r[0]
            ]

        def quaternion_rot(point, q):
            r = [0, point[0], point[1], point[2]]
            q_conj = [q[0], -1*q[1], -1*q[2], -1*q[3]]
            return quaternion_mult(quaternion_mult(q, r), q_conj)[1:]

        def quat_inv(q):
            q_conj = [q[0], -1*q[1], -1*q[2], -1*q[3]]
            q_norm2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]
            return [
                q_conj[0] / q_norm2,
                q_conj[1] / q_norm2,
                q_conj[2] / q_norm2,
                q_conj[3] / q_norm2
            ]


        g = [0, 0, 0, 1]
        q = [self.q0, self.q1, self.q2, self.q3]
        # v1 = quaternion_rot(acc, q)
        # v1 = [round(v, 2) for v in v1]

        # v2 = [v1[0], v1[1], v1[2] - 1.0]
        # # print(v1, v2)
        # v3 = quaternion_rot(v2, quat_inv(q))
        # # print(v3)

        # roll = np.arctan2(self.q0 * self.q1 + self.q2 * self.q3, 0.5 - self.q1 * self.q1 - self.q2 * self.q2)
        # pitch = np.arcsin(-2.0 * (self.q1 * self.q3 - self.q0 * self.q2))
        # yaw = np.arctan2(self.q0 * self.q3 + self.q1 * self.q2, 0.5 - self.q2 * self.q2 - self.q3 * self.q3)

        # return [roll, pitch, yaw, v3[0], v3[1], v3[2]]
        return q

    def updateIMU(self, gyroscope, accelerometer):
        gx, gy, gz = gyroscope
        gx = 0.01745329252  * gx
        gy = 0.01745329252  * gy
        gz = 0.01745329252  * gz

        qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        if norm(accelerometer) != 0:
            # Normalise accelerometer measurement
            accelerometer /= norm(accelerometer)

            ax, ay, az = accelerometer

            _2q0 = 2.0 * self.q0
            _2q1 = 2.0 * self.q1
            _2q2 = 2.0 * self.q2
            _2q3 = 2.0 * self.q3
            _4q0 = 4.0 * self.q0
            _4q1 = 4.0 * self.q1
            _4q2 = 4.0 * self.q2
            _8q1 = 8.0 * self.q1
            _8q2 = 8.0 * self.q2
            q0q0 = self.q0 * self.q0
            q1q1 = self.q1 * self.q1
            q2q2 = self.q2 * self.q2
            q3q3 = self.q3 * self.q3

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * self.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
            s2 = 4.0 * q0q0 * self.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
            s3 = 4.0 * q1q1 * self.q3 - _2q1 * ax + 4.0 * q2q2 * self.q3 - _2q2 * ay

            sarr = [s0, s1, s2, s3]
            snorm = norm(sarr)
            sarr /= snorm
            s0, s1, s2, s3 = sarr

            # Apply feedback step
            qDot1 -= self.beta * s0
            qDot2 -= self.beta * s1
            qDot3 -= self.beta * s2
            qDot4 -= self.beta * s3
        
        self.q0 += qDot1 * self.invSampleFreq
        self.q1 += qDot2 * self.invSampleFreq
        self.q2 += qDot3 * self.invSampleFreq
        self.q3 += qDot4 * self.invSampleFreq

        qarr = [self.q0, self.q1, self.q2, self.q3]
        qnorm = norm(qarr)
        qarr /= qnorm
        self.q0, self.q1, self.q2, self.q3 = qarr

        # roll = np.arctan2(self.q0 * self.q1 + self.q2 * self.q3, 0.5 - self.q1 * self.q1 - self.q2 * self.q2)
        # pitch = np.arcsin(-2.0 * (self.q1 * self.q3 - self.q0 * self.q2))
        # yaw = np.arctan2(self.q0 * self.q3 + self.q1 * self.q2, 0.5 - self.q2 * self.q2 - self.q3 * self.q3)

        q = [self.q0, self.q1, self.q2, self.q3]
        return q
        # return [roll, pitch, yaw]