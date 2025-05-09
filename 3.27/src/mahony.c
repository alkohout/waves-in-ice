/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */


// PURPOSE: calculate wave direction functions
#include <math.h>
#include "util.h"
#include "main.h"
#include "signal_conditioning.h"

#define Kp 2.0*5.0
#define Ki 0.0

double q[4] = {1.0, 0.0, 0.0, 0.0};
double eInt[3] = {0.0, 0.0, 0.0};
float deltat = 0.125;

void mahony_quaternion_update(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz);

// ********************************************************************************
// ********************************************************************************

int mahony(double *gyro, double *accel, double *magno){

    //XXX AK Include testing

    int r=0;

    mahony_quaternion_update(accel[0],accel[1],accel[2],gyro[0],gyro[1],gyro[2],magno[0],magno[1],magno[2]);
    yaw_t   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch_t = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll_t  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    yaw_t   *= 180.0f / pi; 
    yaw_t   -= 17.08674; // Declination at Chukchi sea 14/10/15 at (73.94072, -154.39619) is 17.08674
    yaw_t  *= pi/180.0f;

  
    return(r);

}

void mahony_quaternion_update(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz)
        {
            double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            double norm;
            double hx, hy, bx, bz;
            double vx, vy, vz, wx, wy, wz;
            double ex, ey, ez;
            double pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            double q1q1 = q1 * q1;
            double q1q2 = q1 * q2;
            double q1q3 = q1 * q3;
            double q1q4 = q1 * q4;
            double q2q2 = q2 * q2;
            double q2q3 = q2 * q3;
            double q2q4 = q2 * q4;
            double q3q3 = q3 * q3;
            double q3q4 = q3 * q4;
            double q4q4 = q4 * q4;   

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
        }

