/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "util.h"
#include "main_test.h"

void periodic_func(long double *data, long double period, long double amp, float d_t, int n);
int north_east(struct DATARAW *data_raw, struct RETDATA *return_data);
int wave_direction_FEM_2hz_accel(long double *accel, long double *pitch, long double *roll, struct RETDATA *return_data);

struct DATARAW data_raw;
struct RETDATA return_data;

int test_direction(){

    int i, r=0;
    int period= 12;
    int amp= 1;
    int n = 1280;
    long double degree = 5.0, yaw_mean = -45.0;
    float t=0.0;
    long double kk = 2.0*pi/period;
    long double accel[n], roll[n], pitch[n];
    long double max_roll=-9999, max_pitch=-9999;
    FILE *ido_accel;

    // GENERATE DATA
    return_data.yaw_mean = yaw_mean*pi/180.0;
    return_data.qflg_open_water = 0;
    return_data.qflg_head = 1;
    data_raw.gps_heading=-9999.99;
    periodic_func(data_raw.accel_vert,period,amp,dt,n_raw);
    for (i=0; i<n_raw; i++){
        t += dt;
        data_raw.roll[i] = -0.0*degree*pi/180.0*cos(kk*t); 
        data_raw.pitch[i] = -1.0*degree*pi/180.0*cos(kk*t); 
    }

    // TEST NORTH_EAST
    north_east(&data_raw,&return_data);
    for (i=0;i<n_raw;i++){
         if (max_roll <= data_raw.roll[i]) max_roll = data_raw.roll[i];
         if (max_pitch <= data_raw.pitch[i]) max_pitch = data_raw.pitch[i];
    }
    if ( (max_roll*180/pi<3.54) || (max_roll*180/pi>3.55) ) {
          r = 1;
          debug("max_roll",max_roll*180/pi);
    }
    if ( (max_pitch*180/pi<3.54) || (max_pitch*180/pi>3.55) ) {
          r = 1;
          debug("max_pitch",max_pitch*180/pi);
    }

    // TEST DIRECTION note: testing at 8 Hz
    for (i=0; i<n; i++){
        accel[i] = data_raw.accel_vert[i];
        roll[i] = data_raw.roll[i];
        pitch[i] = data_raw.pitch[i];
    }
    print2out(accel,n,"out/accel.out",ido_accel);
    print2out(pitch,n,"out/pitch.out",ido_pitch);
    print2out(roll,n,"out/roll.out",ido_roll);

    wave_direction_FEM_2hz_accel(accel, pitch, roll, &return_data);
    if ((return_data.peak_direction*180/pi < 44.99) || (return_data.peak_direction*180/pi > 45.01)) {
         r = 1;
         debug("peak_direction",return_data.peak_direction*180/pi);
    }
    if ((return_data.direction*180/pi < 44.99) || (return_data.direction*180/pi > 45.01)) {
         r = 1;
         debug("direction",return_data.direction*180/pi);
    }
    if ((return_data.ratio < 0.5) || (return_data.ratio > 1.5)) {
         r = 1;
         debug("ratio",return_data.ratio);
    }

    return(r);
}

