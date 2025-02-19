/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "util.h"
#include "test_maths.h"

int test_deterministic_analysis(int n, int n_spec, float dt, int k, int m){

    int i, r=0;
    int period= 12;
    int amp= 5;
    long double data[n_spec];
    long double hz_max[2*k], hcm_max[2*k], htm_max[2*k], tz_mean[2*k], hz_mean[2*k], hmod;
    FILE *ido_test;

    periodic_func(data,period,amp,dt,n_spec);
    print2out(data,n_spec,"out/test_data.out",ido_test);
    deterministic_analysis(data,hz_max,hcm_max,htm_max,tz_mean,hz_mean);

    for (i=0; i< 2*k; i++){
         if (hcm_max[i]<amp-0.01) r = 1;  
         if (hcm_max[i]>amp+0.01) r = 1;  
         if (htm_max[i]>-amp+0.01) r = 1;
         if (htm_max[i]<-amp-0.01) r = 1;  
         if (hz_max[i]<2*amp-0.01) r = 1;  
         if (hz_max[i]>2*amp+0.01) r = 1;  
         if ((tz_mean[i]<period-0.01)) r = 1;  
         if ((tz_mean[i]>period+0.01)) r = 1;  
    }

    if (r==1) {
        for (i=0;i<2*k;i++) debug("hcm_max = ",hcm_max[i]);
        for (i=0;i<2*k;i++) debug("htm_max = ",htm_max[i]);
        for (i=0;i<2*k;i++) debug("hz_max = ",hz_max[i]);
        for (i=0;i<2*k;i++) debug("tz_mean = ",tz_mean[i]);
    }

    return(r);
}

