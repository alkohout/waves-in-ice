/* vim: set tfabsltop=8 softtfabsltop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "accel_calib.h"
#include "util.h"
#include "test_maths.h"

int period_end = 4;
int amp_end = 1;

void rspns(long double R[], long double f[], int n, long double f1, long double f2);

int test_decimate(int n, float dt, float dt_raw){

    int r=0;
    long double period, amp;
    
    period = 4;
    amp = 1;
    while ( (r==0) && (period <=period_end) && (amp<=amp_end) ) {
        r = cycle_decimate(period,amp,dt,dt_raw,n);
        if (amp==5){
               amp = 1;
               period++;
        }else amp++;
    }

//    cycle_decimate_tuck(period,1,0.5,0.0625,4096);

    return r;
}

int test_displacement(){

    FILE *idi, *ido_disp;
    int r=0,i;
    int n=2048;
    int n_spec = 5*256;
    int amp=2, period=10;
    float t = -0.5;
    long double maxdisp_processed, maxdisp_true;
    long double accel_in[n];
    long double disp[1280];

    for (i=0; i<n; i++) {
         t += 0.5;
         accel_in[i] = amp*sin(2.0*3.14156*t/period);
    }
    maxdisp_true = pow(period,2)*amp/pow(2.0*3.14156,2);

    intgrt_press(accel_in,disp,.5,n,n_spec,.03,.04);

    for (i=0;i<1280;i++) {
         if (maxdisp_processed <= disp[i]) maxdisp_processed = disp[i];
    }
    if (fabsl(maxdisp_true-maxdisp_processed)>0.05) {
        debug("        Displacement test failed",0);
        debug("        max disp true = ",maxdisp_true);
        debug("        max disp processed = ",maxdisp_processed);
        r = 1;
    }

    return(r);
}

int test_integration(int n, int n_spec, float dt){

    FILE *idi;
    FILE *ido_disp;
    int r=0,i,j;
    int n_spec_2, n2;
    int gamma;
    int strt, end;
    float period, amp;
    long double *data;
    long double *disp;
    long double data_spec[n_spec];
    long double disp_real;
    long double mdr;
    long double max, mean_disp;
    long double max_accel_f, max_accel_s;
    long double max_disp_f, max_disp_s;
    long double data_amp[85];
    long double f1 = 0.03;
    long double f2 = 0.04;
    long double win[n_spec];
    long double alpha, beta, t1;
    
    // Allocate pointers
    data = (long double*) malloc(n*sizeof(long double));
    disp = (long double*) malloc(n_spec*sizeof(long double));

    period = 10.0;
    amp = 1.0;

//    idi = fopen("in/disp_amp.in","r");
//	if (idi == NULL)
//		perror("Error disp_amp.in");
//    if (idi == NULL) {
//         debug("\nCan not open data_amp.in..\n", 0);
//    }else{
//         for ( i=0; i<85; i++) fscanf(idi,"%Le\n",&data_amp[i]);
//    }
//    fclose(idi);

    periodic_func(data_spec,period,amp,dt,n_spec);
    
//  Apply window function (%10 cosine taper)
    gamma = 5; t1 = n_spec/2-(n_spec/2)/gamma; alpha = 2.0*3.14159265*gamma/n_spec;
    if (gamma % 2 == 0) beta = 3.14159265; else beta = 0;
    for (i=0;i<n_spec/2;i++) {
         if (i<t1) win[i] = 1;
         else win[i] = 0.5*(1 + cos(alpha*i + beta)); 
    }
    for (i=n_spec/2;i<n_spec;i++) win[i] = win[i-n_spec/2]; 
    for (i=0;i<n_spec/2;i++) win[i] = win[n_spec-1-i];
    for (i=0;i<n_spec;i++) data_spec[i] = win[i]*data_spec[i];

//  Add zeros to make power of 2
    strt = (n-n_spec)/2;
    end = n-strt;
    for (i=0;i<strt;i++) data[i] = 0.0;
    for (i=strt;i<end;i++) data[i] = data_spec[i-strt];
    for (i=end;i<n;i++) data[i] = 0.0;                   

//    for (i=n_spec; i<n; i++) data[i]=0.0;
    disp_real = pow(period,2)/pow(2*3.14159,2);

    // peak freak accel
    r = peak_freq(data,&max_accel_f,&max_accel_s,dt,n);

    // integrate
    mdr = intgrt_press(data,disp,dt,n,n_spec,f1,f2);
    print2out(disp,n_spec,"out/int_test_disp.out",ido_disp);

    // peak freak
    n_spec_2=n_spec;
    n2 = n_spec_2;
    while (((n2%2)==0) && n2 > 1) n2 /=2;
    if(n2!=1) { 
        while (n2!=1) {
         n2 = n_spec_2;
         while (((n2%2)==0) && n2 > 1) n2 /=2;
         if(n2!=1) n_spec_2 ++;
        }
        n_spec_2 /=2;
    }
    r = peak_freq(disp,&max_disp_f,&max_disp_s,dt,n_spec_2);
    if (fabsl(max_disp_f - 1/period) > 0.05 ){
        debug("        peak frequency test failed",0);
        r=1;
    }
    // amplitude 
    max = 0.0;
    for (i=0; i<n_spec; i++) if (disp[i] > max) max = disp[i];

    if (fabsl(max - disp_real) > 0.05 ){
        debug("        amplitude test failed",0);
        r=1;
    }
    if ( (verbose_error && r == 1) || (verbose) ){
        debug("      frequency    ", 1/period);
        debug("        max accel f", max_accel_f);
        debug("        max disp f ", max_disp_f);
        debug("      amp          ", amp);
        debug("        disp amp   ", disp_real);
        debug("        max        ", max);
    }

    free(data);
    free(disp);

    return r;

}

int test_sample_length(){

    int r=0;
    int i,n,n_spec;
    float dt = 0.5;

    for (i=10;i<=12;i++){  
        n = pow(2,i);
        n_spec = n-50;

        debug("\n            start decimate test",0);
        r = test_decimate(n,0.5,0.015625);
        if (r==1) { 
           debug("            decimate test failed for n = ",n);
        }
        else {
           debug("            decimate test passed for n = ",n);
        }

        debug("\n            start integration test",0);
        r = test_integration(n,n_spec,dt);
        if (r) { 
           debug("            integration test failed for n = ",n);
        } 
        else {
           debug("            integration test passed for n = ",n);
        }
    }

    return r;
}

int test_dt(){

    int r=0;
    int m=512;
    int i,n=4096,n_spec=5*m;
    float dt;
    float dt_raw;
    
    debug("\n            start decimate test - vary dt",0);
    dt_raw = 0.015625;
    dt = dt_raw*4;
    while (dt<=0.5){
        r = test_decimate(n,dt,dt_raw);
        if (r) { 
           debug("            decimate test failed for dt = ",dt);
        } else {
           debug("            decimate test passed for dt = ",dt);
        }
        dt *= 4;
    }
    
    debug("\n            start integration test - vary dt",0);
    dt = 0.125;
    while (dt < .8) {
        r = test_integration(n,n_spec,dt);
        if (r) { 
           debug("            integration test failed for dt = ",dt);
        } else {
           debug("            integration test passed for dt = ",dt);
        }
        dt*=2;
    }

    return r;
}

int test_spectrum(){

    int r=0, i, m, n, k = 2;
    int tdt=0,tm=0,tp=0,ta=0;
    int tdtf=0,tmf=0,tpf=0,taf=0;
    long double period, amp;
    float dt;


    debug("\n            start spectrum test - vary dt ",0);
    m = 256;
    n = 5*m;
    period = 10.0;
    amp = 1.0;
    for(dt=.25;dt<0.8;dt*=2){ 
        tdt = 0;
        tdt = test_spectrum_cycle(period, amp, dt, m); 
        if (tdt==1) tdtf = 1;
        if (tdt==1) {
            debug("                spectrum test failed for dt = ",dt);
            debug("\n",0);
        }
    }
    if (tdtf==0) debug("            spectrum test - vary dt passed",0);

    debug("\n            start spectrum test - vary m ",0);
    for(m=256;m<256*5;m*=2){ 
        tm=0;
        tm = test_spectrum_cycle(period, amp, dt, m); 
        if (tm==1) tmf = 1;
        if (tm==1) {
            debug("                spectrum test failed for m = ",m);
            debug("\n",0);
        }
    }
    if (tmf==0) debug("            spectrum test - vary m passed",0);

    debug("\n            start spectrum test - vary period",0);
    m=256;
    for(period = 4;period<20;period+=2){ 
        tp=0;
        tp = test_spectrum_cycle(period, amp, dt, m); 
        if (tp==1) tpf = 1;
        if (tp==1) { 
            debug("                spectrum test failed for period = ",period);
            debug("\n",0);
        }
    }
    if (tpf==0) debug("            spectrum test - vary period passed",0);

    debug("\n            start spectrum test - vary amp",0);
    m=256;
    period = 10;
    for(amp = .01;amp<2;amp+=.5){ 
        ta=0;
        ta = test_spectrum_cycle(period, amp, dt, m); 
        if (ta==1) taf = 1;
        if (ta==1) {
            debug("                spectrum test failed for amp = ",amp);
            debug("\n",0);
        }
    }
    if (taf==0) debug("            spectrum test - vary amp passed",0);

    if (tdtf || tmf || tpf || taf != 0) r = 1;

    return r;

}

void periodic_func(long double *data, long double period, long double amp, float dt, int n){

    int i;
    float t = 0;
    long double kk = 2.0*pi/period;

    for (i=0; i<n; i++){
        t += dt;
        data[i] = (float)amp*sin(kk*t); 
    }

}

int peak_freq(long double *data, long double *max_f, long double *max_s, float dt, int n){

    logger("Start peak freq in tm_misc.c",0.0);

    int r=0;
    int i,j;
    int max_s_id;
    long double f[n];
    long double in[n];
    long double in_fabsl[n/2];

    //long double *in = calloc(n,sizeof(long double));
    //if (in == NULL) printf("Couldnt allocate memory in in in peak_freq in tm_misc.c");
    logger("    Declarations complete",0.0);

    // f domain
    for (i=0; i<n; i++){
           f[i] = i/(n*dt);
    }

    //realft
    for (i = 0; i<n; i++){
        in[i] = data[i];
    }
    realft(in,n,1);

    //locate peak 
    in_fabsl[0] = in[0];
    in_fabsl[n/2] = in[1];
    for (i=2,j=0; i<n; i+=2, j++) {
           in_fabsl[j] = sqrt(pow(in[i],2) + pow(in[i+1],2));
    }
    *max_s = 0.0;
    for (i=0; i<j+1; i++) {
           if (in_fabsl[i] > *max_s){
                  *max_s = in_fabsl[i];
                  max_s_id = i; 
           }
    }

    *max_f = f[max_s_id];

  return r;

}


