/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "util.h"
#include "test_maths.h"

int test_detrend(){

    int i = 0;
    int r = 0;
    int n = 32768;

    long double period=10.0;
    long double amp = 1.0;
    float dt = .5;
    float t = 0;
    float y;

    long double data[n];
    long double mean;

    FILE *ido_trend;
    FILE *ido_detrend;
    FILE *idi_accel;

    // test periodic function
    periodic_func(data,period,amp,dt,n);
    for (i=0; i<n; i++){
         y = .01*t + 2;
         t = t + dt;
         data[i] = data[i] + y;
    }

    for (i=0; i<n; i++) {
         data[i] += 0.02;
         mean += data[i];
    }
    mean /= n;

    ido_trend = fopen("out/trend.out","w");
	if (ido_trend == NULL)
	  perror("Error trend.out");
    for ( i=0; i<n; i++) {
         fprintf(ido_trend,"%LF\n",data[i]);
    }
    fclose(ido_trend);
    
    //demean(data,n);
    r = trend_removal(data,n);
    ido_detrend = fopen("out/detrend.out","w");
	if (ido_detrend == NULL)
	  perror("Error detrend.out");
    for ( i=0; i<n; i++) {
         fprintf(ido_detrend,"%LF\n",data[i]);
    }
    fclose(ido_detrend);

    return r;
}

// ********************************************************************************
// ********************************************************************************

int test_despike(){

       int i, r = 0, qflg=0;
       int n = 10005;
       int st, sm;
       int rndm=0,indx; 

       float dt = 0.5;

       long double data[n];
       long double period=10.0;
       long double amp = 1.0;
       long double spk,rndm2;
       
       FILE *ido_spike;

       // test periodic function
       periodic_func(data,period,amp,dt,n);
       for (i=0; i<20; i++){
            rndm = rndm + 500;
            data[rndm] = amp*10;
       }
       data[rndm+1] = data[rndm];
       data[rndm+2] = data[rndm];

       demean(data,n);
       despike(data,&st,&sm,n,6);

       if (st != 22) r = 1;
       if (sm != 3) r = 1;

       return(r);
}
