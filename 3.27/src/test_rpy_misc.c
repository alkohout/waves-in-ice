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

int test_rpy_static(int roll, int pitch){

    int r_static=0;
    int i,j,n=16384;
    double accel=0;
    double rt, pt;
    double accel_e;

    // initialise gyro, accel, magno  
//    for (i=0; i<3; i++){ 
//        gyro[i] = 0; 
//  	acc[i] = 0; 
//  	magno[i] = 0; 
//    } 
       
//    rt = roll*pi/180.0; pt = pitch*pi/180.0; accel_e = 0; 
//    acc[0] = -sin(pt)*accel_e;
//    acc[1] = sin(rt)*cos(pt)*accel_e;
//    acc[2] = cos(rt)*cos(pt)*accel_e;
//    for (j=0; j<n; j++) rpy(gyro,acc,magno); 
//    accel = accel_orient_approx(acc[2],roll_t,pitch_t);
//    if (fabs(accel)>0.0001) r_static = 1;

    return r_static;

}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

double test_rpy_z_accuracy(){
    
    FILE *id;
    int i=0,j;
    double roll,pitch;
    double accel[10000];
    double accelo[10000];
    double accele[10000];
    double rt, pt;
    double accel_e=1.0001*g;
    double er = 20*pi/180;
    double a[3];

//    for (roll=0;roll<=99;roll++){
//    for (pitch=0;pitch<=99;pitch++){
//         rt = roll*pi/180.0; pt = pitch*pi/180.0;  
//         a[0] = (round(-sin(pt)*accel_e/g*10000))/10000;
//         a[1] = (round(sin(rt)*cos(pt)*accel_e/g*10000))/10000;
//         a[2] = cos(rt)*cos(pt)*accel_e/g;
//         accel[i] = a[2]*g;
//         for (j=0;j<3;j++) a[j] *= g;
//         accelo[i] = accel_orient_approx(a[2],rt+er,pt+er);
//         accele[i] = accel_orient_exact(a,rt+er,pt+er);
 //        i++;
 //   }}
 //   print2out(accel,10000,"out/rpy_z_accuracy.out",id);
 //   print2out(accelo,10000,"out/rpy_z_orient.out",id);
 //   print2out(accele,10000,"out/rpy_z_exact.out",id);
    
    return 0.0;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

    /*
    debug("\n            start static tests",0);

    rt = 80*pi/180.0;
    pt = 80*pi/180.0;
    accel_e = indx*g; 

    acc[0] = -sin(pt)*accel_e;
    acc[1] = sin(rt)*cos(pt)*accel_e;
    acc[2] = cos(rt)*cos(pt)*accel_e;
    acc[0] = round(acc[0]);
    acc[1] = round(acc[1]);
    acc[2] = round(acc[2]*10)/10;

    accel_kist = g*(1-acc[2]/1000);
    accel_real = g*(1 - accel_e/1000);

    for (j=0; j<n; j++) rpy(gyro,acc,magno); 
    accel_kist_rp = accel_orient(acc,roll_t,pitch_t);
    accel_imu = g*(1-(-sin(pitch_t)*acc[0] + 
                             sin(roll_t)*cos(pitch_t)*acc[1] + 
                             cos(roll_t)*cos(pitch_t)*acc[2])/1000);

    if (fabs(accel_imu - accel_real)>0.001) r_accel = 1;

    if (r_accel) debug("            accelerating tests failed",0); 
    if (!r_accel) debug("            accelerating tests passed",0);
    */


