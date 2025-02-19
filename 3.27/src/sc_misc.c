/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */


// PURPOSE: Misc maths functions
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "util.h"
#include "main.h"
#include "signal_conditioning.h"

// ********************************************************************************
// ********************************************************************************

void calibrate(struct DATARAW *data_raw){

    calibrate_kist(data_raw->accel_raw_kist,n_raw);
    calibrate_accel(data_raw->accel_raw_x,n_raw);
    calibrate_accel(data_raw->accel_raw_y,n_raw);
    calibrate_accel(data_raw->accel_raw_z,n_raw);
    calibrate_gyro(data_raw->gyro_raw_x,n_raw);
    calibrate_gyro(data_raw->gyro_raw_y,n_raw);
    calibrate_gyro(data_raw->gyro_raw_z,n_raw);
    calibrate_magno(data_raw->magno_raw_x,n_raw);
    calibrate_magno(data_raw->magno_raw_y,n_raw);
    calibrate_magno(data_raw->magno_raw_z,n_raw);
    calibrate_pos(data_raw->roll,n_raw);
    calibrate_pos(data_raw->pitch,n_raw);
    calibrate_pos(data_raw->yaw,n_raw);

    return;

}

void calibrate_kist(long double *data, int n){
       // Remove slope and intercept and return in m/s 
       int i;
       for (i=0; i<n; i++) data[i] = 9.80665*((data[i] - calib[0]) / calib[1]); 
       return;
}

void calibrate_accel(long double *data, int n){
       // Remove slope and intercept and return in m/s 
       int i;
       //for (i=0; i<n; i++) data[i] = 9.80665*((data[i] - calib[2]) / calib[3]); 
       for (i=0; i<n; i++) data[i] *= 9.80665/10000;
       return;
}

void calibrate_gyro(long double *data, int n){
       // Return in radians / second
       int i;
       for (i=0; i<n; i++) data[i] *= pi/180.0/1000.0;   
       return;
}

void calibrate_magno(long double *data, int n){
       // Return's in mili gauss
       int i;
       for (i=0; i<n; i++) data[i]/=1000.0;
       return;
}

void calibrate_pos(long double *data, int n){
       // Return in radians
       int i;
       for (i=0; i<n; i++) data[i] *= pi/(180.0*100.0);
       return;
}

// ********************************************************************************
// ********************************************************************************

void gpsraw_2_gps8hz(struct DATARAW *data_raw, struct RETDATA *return_data){
     
     int i,j,ii,st;
     int temp1,temp2;
     unsigned long long tm;

     tm = data_raw->start;
     st = 0;
     // initialize gps_headings_8hz for patching
     for (ii=0; ii<n_raw; ii++) data_raw->gps_headings_8hz[ii] = 9999.0;
     // match gps headings with start time
     for (ii=0; ii<n_raw; ii++){
          for (i=st; i<return_data->n_gps; i++){
               if (tm == data_raw->start) j=0;
               if (data_raw->gps_time[i] == tm) {
                   data_raw->gps_headings_8hz[j] = data_raw->gps_headings[i]; 
                   st = i;
                   break;
               }
          }
          tm++;
          j+=8;
     }

     // interpolate gaps
     patch(data_raw->gps_headings_8hz,n_raw);

     print2out(data_raw->gps_headings_8hz,n_raw,"out/gps_headings.out",ido_gps);
}

// ********************************************************************************
// ********************************************************************************

void decimate(long double *in, long double *out, float sr1, float sr2, int n1, int n2) {

       logger("Start decimate",0.0);

       int i, j, ii, n, jj;
       int step;
       long double out1[n1];
//       long double fout1[n1];
       long double a[3],b[3];

       logger("    n1",n1);
       // intialize out
       for (i=0;i<n2;i++) out[i] = 0.0;

       // butter worth filter with 8 Hz sample rate and 0.5 Hz cut off
       if (sr1==0.125) {
           logger("    Butter worth filter with 8 Hz sample rate, 0.5 Hz cut off",0.0);
           a[0] = 1.000000000000000;
           a[1] = -1.454243586251585;
           a[2] = 0.574061915083955;
           b[0] = 0.029954582208092;
           b[1] = 0.059909164416185;
           b[2] = 0.029954582208092;
       }
       // butter worth filter with 64 Hz sample rate and 2 Hz cut off
       if (sr1==0.015625) {
           logger("    Butter worth filter with 64 Hz sample rate, 2 Hz cut off",0.0);
           a[0] = 1.000000000000000;
           a[1] = -1.723776172762509;
           a[2] = 0.757546944478829;
           b[0] = 0.008442692929080;
           b[1] = 0.016885385858160;
           b[2] = 0.008442692929080;
       }
       // butter worth filter with 64 Hz sample rate and 1 Hz cut off
//       if (sr1==0.015625) {
//           a[0] = 1.000000000000000;
//           a[1] = -1.861361146829083;
//           a[2] = 0.870367477456469;
//           b[0] = 0.002251582656847;
//           b[1] = 0.004503165313693;
//           b[2] = 0.002251582656847;
//       }

       if (!floorf(sr2/sr1==sr2/sr1)) {
              debug("Error: step a non integer",0.0);
              return;
       }  

       out1[0] = in[0];
       out1[1] = in[1];
       for (i=2; i<n1; i++){
            out1[i] = b[0]*in[i] +  b[1]*in[i-1] + b[2]*in[i-2] 
                                - a[1]*out1[i-1] - a[2]*out1[i-2];
       } 
//       for (i=0; i<n1; i++) fout1[i] = out1[i];
//       realft(fout1,(unsigned long)n1,1);
//       fout1[0] = 2.0/n1*pow(fout1[0],2); fout1[n1/2] = 2.0/n1*pow(fout1[1],2);
//       for (i=2,jj=1; i<n1/2-1; i+=2,jj++) {
//              fout1[jj] = 2.0/n1*(pow(fout1[i],2) + pow(fout1[i+1],2));
//       }
       
       step = sr2/sr1;
       logger("    step",step);
       for (i=0, j=0; i<n1; i+=step,j++){
              if (j>=n2) {
                  fprintf(stderr,"ERROR: out of bounds in decimate. Exiting.\n");
                  exit(0);
              }
              out[j] = out1[i];
       } 

       logger("End decimate",0.0);
       return;
}

// ********************************************************************************
// ********************************************************************************

void accel_orient_exact_imu(struct DATARAW *data_raw, struct RETDATA *return_data) {

       int i;

       #ifdef ARCTIC
          logger("ARCTIC defined. Roll and pitch defined using Arduino software",0.0);
          mean_rpy(data_raw,return_data);
          for (i=0;i<n_raw;i++){ 
               data_raw->accel_vert[i] =  -sin(data_raw->pitch_arduino+data_raw->pitch[i])*data_raw->accel_raw_x[i] + 
                         sin(data_raw->roll_arduino+data_raw->roll[i])*cos(data_raw->pitch_arduino+data_raw->pitch[i])*data_raw->accel_raw_y[i] + 
                         cos(data_raw->roll_arduino+data_raw->roll[i])*cos(data_raw->pitch_arduino+data_raw->pitch[i])*data_raw->accel_raw_z[i];
           }
       #else
          for (i=0;i<n_raw;i++){ 
               data_raw->accel_vert[i] =  -sin(data_raw->pitch[i])*data_raw->accel_raw_x[i] + 
                         sin(data_raw->roll[i])*cos(data_raw->pitch[i])*data_raw->accel_raw_y[i] + 
                         cos(data_raw->roll[i])*cos(data_raw->pitch[i])*data_raw->accel_raw_z[i];
          }
       #endif
       
}

// ********************************************************************************
// ********************************************************************************
void accel_orient_exact_kist(struct DATARAW *data_raw, struct RETDATA *return_data) {

       int i;

       #ifdef ARCTIC
          logger("ARCTIC defined. Roll and pitch defined using Arduino software",0.0);
          mean_rpy(data_raw,return_data);
          for (i=0;i<n_raw;i++){ 
               data_raw->accel_vert[i] =  -sin(data_raw->pitch_arduino+data_raw->pitch[i])*data_raw->accel_raw_x[i] + 
                         sin(data_raw->roll_arduino+data_raw->roll[i])*cos(data_raw->pitch_arduino+data_raw->pitch[i])*data_raw->accel_raw_y[i] + 
                         cos(data_raw->roll_arduino+data_raw->roll[i])*cos(data_raw->pitch_arduino+data_raw->pitch[i])*data_raw->accel_raw_kist[i];
           }
       #else
          for (i=0;i<n_raw;i++){ 
               data_raw->accel_vert[i] =  -sin(data_raw->pitch[i])*data_raw->accel_raw_x[i] + 
                         sin(data_raw->roll[i])*cos(data_raw->pitch[i])*data_raw->accel_raw_y[i] + 
                         cos(data_raw->roll[i])*cos(data_raw->pitch[i])*data_raw->accel_raw_kist[i];
          }
       #endif
        
}

// ********************************************************************************
// ********************************************************************************

long double accel_orient_approx(long double a, long double r, long double p) {

       int i;
       long double a_out;

           // calculate in g's
           a /= g;
           // return on m/s2
           a_out = g*(a-cos(r)*cos(p))*cos(r)*cos(p) + g;

       return a_out;

}

// ********************************************************************************
// ********************************************************************************

void mean_rpy(struct DATARAW *data_raw, struct RETDATA *return_data){

    #ifdef ARCTIC
    #ifdef DECLINATION

    int j;
    double acc[3],magno[3],gyro[3];
    long double roll_arduino[n_raw],pitch_arduino[n_raw],yaw_arduino[n_raw]; 
    long double roll_mahony[n_raw],pitch_mahony[n_raw],yaw_mahony[n_raw]; 
    long double ra=0.0,pa=0.0,ya=0.0;

    for (j=0; j<n_raw; j++){
         // allocate gyro, accel, magno  
         acc[0] = (double) data_raw->accel_raw_x[j];
         acc[1] = (double) data_raw->accel_raw_y[j];
         acc[2] = (double) data_raw->accel_raw_z[j];
         magno[0] = (double) data_raw->magno_raw_x[j];
         magno[1] = (double) data_raw->magno_raw_y[j];
         magno[2] = (double) data_raw->magno_raw_z[j];
         gyro[0] = (double) data_raw->gyro_raw_x[j];
         gyro[1] = (double) data_raw->gyro_raw_y[j];
         gyro[2] = (double) data_raw->gyro_raw_z[j];

	 // retrieve roll, pitch, yaw  at each time step
	 rpy(gyro,acc,magno); 
         roll_arduino[j] = roll_t; 
         pitch_arduino[j] = pitch_t; 
         yaw_arduino[j] = yaw_t - declination*pi/180;
         data_raw->yawa[j] = yaw_t - declination*pi/180;

         if (j>5000){
             ra += roll_arduino[j];
             pa += pitch_arduino[j];
             ya += yaw_arduino[j];
         }

    } // for i = 0:n_raw

    data_raw->roll_arduino = ra/(n_raw-5000); 
    data_raw->pitch_arduino = pa/(n_raw-5000);  
    data_raw->yaw_arduino = ya/(n_raw-5000);

    print2out(roll_arduino,n_raw,"out/rolla.out",ido_rolla);
    print2out(pitch_arduino,n_raw,"out/pitcha.out",ido_pitcha);
    print2out(yaw_arduino,n_raw,"out/yawa.out",ido_yawa);

    #endif
    #endif

}



