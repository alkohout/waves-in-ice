/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "accel_calib.h"
#include "util.h"
#include "test_maths.h"
#include "main.h"
#include "signal_conditioning.h"

int test_rpy_static(int roll, int pitch);
double test_rpy_z_accuracy();

int test_rpy(){

    int r=0;
    int i,j;
    int roll, pitch;

    double kist, gyro[3], acc[3], magno[3];

    // initialise gyro, accel, magno  
    for (i=0; i<3; i++){ 
        gyro[i] = 0; 
  	acc[i] = 0; 
  	magno[i] = 0; 
    } 

    debug("\n            start static tests",0);
    for (roll=0;roll<=90;roll+=10){
    for (pitch=0;pitch<=90;pitch+=10){
         r = test_rpy_static(roll,pitch);
    }}

    kist = test_rpy_z_accuracy();

    return r;

}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

int test_rpy_5min() {

    FILE *id;

    int i, j;
    int r = 0;
    int n = 2394;

    double accel_raw[n], accel_x[n], accel_y[n], accel_z[n]; 
    double mag_x[n], mag_y[n], mag_z[n];
    double gyro_x[n], gyro_y[n], gyro_z[n]; 
    double acc[3], magno[3], gyro[3];
    double roll[n], pitch[n], yaw[n];

    // open raw data
    FILE *idi_raw;
    idi_raw = fopen("in/dig_raw.in","r");
	if (idi_raw == NULL)
		perror("Error dig_raw.in");
    for ( i=0; i<n; i++) {
        fscanf(idi_raw,"%*f%*c%lf%*c%lf%*c%lf%*c%lf%*c%lf%*c%lf%*c%lf%*c%lf%*c%lf%*c%lf%*c\n\n",&accel_raw[i],&accel_x[i],&accel_y[i],&accel_z[i],&mag_x[i],&mag_y[i],&mag_z[i],&gyro_x[i],&gyro_y[i],&gyro_z[i]);
    }
    fclose(idi_raw);
  
    // open calibration file
    FILE *idi_calib;
    idi_calib = fopen("in/calibrate.in","r");
	if (idi_calib == NULL)
		perror("Error calibrate.in");
    for ( i=0; i<n_calib; i++) fscanf(idi_calib,"%*s%lf",&calib[i]); 
    fclose(idi_calib);

    //calibrate_accel(accel_x,n,-1);
    //calibrate_accel(accel_y,n,-1);
    //calibrate_accel(accel_z,n,-1);
    //calibrate_mag(mag_x,n,calib[2],calib[3],1);
    //calibrate_mag(mag_y,n,calib[4],calib[5],1);
    //calibrate_mag(mag_z,n,calib[6],calib[7],1);
    //calibrate_gyro(gyro_x,n,1);
    //calibrate_gyro(gyro_y,n,1);
    //calibrate_gyro(gyro_z,n,1);
    
  //  for (j=0; j<n; j++){
  //      // allocate gyro, accel, magno  
  //      acc[0] = accel_x[j];
  //      acc[1] = accel_y[j];
  //      acc[2] = accel_z[j];
  //      magno[0] = mag_x[j];
  //      magno[1] = mag_y[j];
  //      magno[2] = mag_z[j];
  //      gyro[0] = gyro_x[j];
  //      gyro[1] = gyro_y[j];
  //      gyro[2] = gyro_z[j];
        // retrieve roll, pitch, yaw  at each time step
  //      rpy(gyro,acc,magno); 
  //      roll[j] = roll_t; 
  //      pitch[j] = pitch_t; 
  //      yaw[j] = yaw_t;
//    } 
  //  print2out(roll,n,"out/roll_5min.out",id);
  //  print2out(pitch,n,"out/pitch_5min.out",id);
  //  print2out(yaw,n,"out/yaw_5min.out",id);
                  
    return r;
}
