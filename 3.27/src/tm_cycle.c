/* vim: set tfabsltop=8 softtfabsltop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "accel_calib.h"
#include "util.h"
#include "test_maths.h"
#include <string.h>

int cycle_decimate(long double period, long double amp, float dt, float dt_raw, int n) { 

    int r=0;
    int i,j;
    float rndm;
    float noise_level = 0.25;
    int n_raw = (int)(n/(dt_raw/dt));
    
    long double *in;
    long double *out;
    long double *in_noise, *out_noise;
    long double mean_in=0.0, mean_out=0.0;
    long double mean_in_noise=0.0, mean_out_noise=0.0;
    long double max_in=0.0, max_out=0.0;
    long double max_fin=0.0, max_fout=0.0;
    long double max_sin=0.0, max_sout=0.0;
    long double max_in_noise=0.0, max_out_noise=0.0;
    long double max_fin_noise=0.0, max_fout_noise=0.0;
    long double max_sin_noise=0.0, max_sout_noise=0.0;

    // Allocate pointers 
    in = (long double*) malloc(n_raw*sizeof(long double));
    out = (long double*) malloc(n*sizeof(long double));
    in_noise = (long double*) malloc(n_raw*sizeof(long double));
    out_noise = (long double*) malloc(n*sizeof(long double));

    // generate in data
    periodic_func(in,period,amp,dt_raw,n_raw);
    
    // generate noise
    rndm = rand()%100;
    for (i = 0; i<n_raw; i++) in_noise[i] = in[i] + amp*noise_level*rndm/100.0;

    // calculate means
    for (i = 0; i<n_raw; i++){
       mean_in = mean_in + in[i];
       mean_in_noise = mean_in_noise + in_noise[i];
       if (in[i] > max_in) max_in = in[i];
       if (in_noise[i] > max_in_noise) max_in_noise = in_noise[i];
    }
    mean_in = mean_in/n_raw;
    mean_in_noise = mean_in_noise/n_raw;

    // simple out 
    decimate(in,out,dt_raw,dt,n_raw,n);
    logger("decimate in and out complete",0.0);

    for (i=0; i<n; i++){
       mean_out = mean_out + out[i];
       if (out[i] > max_out) max_out = out[i];
    }

    // noise out
    decimate(in_noise,out_noise,dt_raw,dt,n_raw,n);
    logger("decimate in_noise and out_noise complete",0.0);
    for (i=0; i<n; i++){
       mean_out_noise = mean_out_noise + out_noise[i];
       if (out_noise[i] > max_out_noise) max_out_noise = out_noise[i];
    }
    mean_out = mean_out/n;
    mean_out_noise = mean_out_noise/n;
    logger("mean out complete",0.0);

    // calculate peak frequency and spectral max
    peak_freq(in, &max_fin, &max_sin, dt_raw, n_raw);
    logger("in peak freq complete",0.0);
    peak_freq(in_noise, &max_fin_noise, &max_sin_noise, dt_raw, n_raw);
    logger("in_noise peak freq complete",0.0);
    peak_freq(out, &max_fout, &max_sout, dt, n);
    logger("out peak frequency complete",0.0);
    peak_freq(out_noise, &max_fout_noise, &max_sout_noise, dt, n);
    logger("out_noise peak frequency complete",0.0);
    
    // mean test
    if ( (fabsl(mean_in-mean_out)>0.001) || (mean_out > 0.01) ){
       debug("        simple mean test failed",0);
       r=1;
    }
    if ( (fabsl(mean_in_noise-mean_out_noise)>0.001) || (mean_out_noise > amp/4.0) ){
       debug("        noise mean test failed",0);
       r=1;
    }
    logger("mean test complete",0.0);

    // amplitude test
    if ( fabsl(max_in - max_out) > 1.0) {
       debug("        simple amplitude test failed",0);
       r=1;
    }
    if ( fabsl(max_in_noise - max_out_noise) > 1.0) {
       debug("        noise amplitude test failed",0);
       r=1;
    }
    logger("amplitude test complete",0.0);

    // compare maxfin with maxfout
    if (fabsl(max_fin-max_fout) > 0.01) {
       debug("        simple frequency test failed",0);
       r=1;
    }
    if (fabsl(max_fin_noise-max_fout_noise) > 0.01) {
       debug("        noise frequency test failed",0);
       r=1;
    }
    logger("maxfin and maxfout test complete",0.0);

    // test height
    if (fabsl(max_sin/n_raw - max_sout/n) > 0.1) {
       debug("        simple spectrum test failed",0);
       r=1;
    }
    if (fabsl(max_sin_noise/n_raw - max_sout_noise/n) > 0.1) {
       debug("        noise spectrum test failed",0);
       r=1;
    } 
    logger("height test complete",0.0);

    if ( (verbose_error && r==1) || (verbose) ){
       debug("           period",period);
       debug("           amp   ",amp);
       debug("              mean_in         ",mean_in);
       debug("              mean_out        ",mean_out);
       debug("              mean_in_noise   ",mean_in_noise);
       debug("              mean_out_noise  ",mean_out_noise);
       debug("              max_in          ",max_in);
       debug("              max_out         ",max_out);
       debug("              max_in_noise    ",max_in_noise);
       debug("              max_out_noise   ",max_out_noise);
       debug("              max_fin         ",max_fin);
       debug("              max_fout        ",max_fout);
       debug("              max_fin_noise   ",max_fin_noise);
       debug("              max_fout_noise  ",max_fout_noise);
       debug("              max_spectrum in ",max_sin/n_raw);
       debug("              max_spectrum out",max_sout/n);
       debug("              max_spectrum in ",max_sin_noise/n_raw);
       debug("              max_spectrum out",max_sout_noise/n);
    }

    for (i=0; i<n_raw; i++) in[i] = 0.0;
    for (i=0; i<n_raw; i++) in_noise[i] = 0.0;
    for (i=0; i<n; i++) out[i] = 0.0;
    for (i=0; i<n; i++) out_noise[i] = 0.0;

    free(in);
    free(out);
    free(in_noise);
    free(out_noise);

    return r;
}

//int cycle_decimate_tuck(float period, float amp, float dt, float dt_raw, int n) { 

//    int r,i,j;
//    int m = 512, m_raw = 4096;
//    int k = 2;
//    int n_raw = (int)(n/(dt_raw*2));
//    double psd[m], psdo[m], psd_raw[m_raw];
//    double f[m], f_raw[m_raw];
    
//    double in[n_raw]; 
//    double in1[n_raw]; 
//    double in2[n_raw]; 
//    double in3[n_raw]; 
//    double in4[n_raw]; 
//    double in5[n_raw]; 
//    double in6[n_raw]; 
//    double in7[n_raw]; 
//    double in8[n_raw]; 
//    double in9[n_raw]; 
//    double in10[n_raw]; 
//    double in11[n_raw]; 
//    double in12[n_raw]; 
//    double in13[n_raw]; 
//    double in14[n_raw]; 
//    double in15[n_raw]; 
//    double in16[n_raw]; 
//    double in17[n_raw]; 
//    double in18[n_raw]; 
//    double in19[n_raw]; 
//    double in20[n_raw]; 
//    double out[n];
//    double outo[n];
//    double max_fin=0.0, max_fout=0.0;
//    double max_sin=0.0, max_sout=0.0;
//
//    FILE *ido_dpsd;
//    FILE *ido_dpsdo;
//    FILE *ido_df;
//    FILE *ido_dpsdin;
//    FILE *ido_dfin;
//
//    // generate in data
//    periodic_func(in1,18,amp,dt_raw,n_raw);
//    periodic_func(in2,10.0,amp,dt_raw,n_raw);
//    periodic_func(in3,8.0,amp,dt_raw,n_raw);
//    periodic_func(in4,7.0,amp,dt_raw,n_raw);
//    periodic_func(in5,6.0,amp,dt_raw,n_raw);
//    periodic_func(in6,5.0,amp,dt_raw,n_raw);
//    periodic_func(in7,4.0,amp,dt_raw,n_raw);
//    periodic_func(in8,3.0,amp,dt_raw,n_raw);
//    periodic_func(in9,2.0,amp,dt_raw,n_raw);
//    periodic_func(in10,1.0,amp,dt_raw,n_raw);
//    periodic_func(in11,1/1.1,amp,dt_raw,n_raw);
//    periodic_func(in12,1/1.2,amp,dt_raw,n_raw);
//    periodic_func(in13,1/1.3,amp,dt_raw,n_raw);
//    periodic_func(in14,1/1.4,amp,dt_raw,n_raw);
//    periodic_func(in15,1/1.5,amp,dt_raw,n_raw);
//    periodic_func(in16,1/1.6,amp,dt_raw,n_raw);
//    periodic_func(in17,1/1.7,amp,dt_raw,n_raw);
//    periodic_func(in18,1/1.8,amp,dt_raw,n_raw);
//    periodic_func(in19,1/1.9,amp,dt_raw,n_raw);
//    periodic_func(in20,1/2.0,amp,dt_raw,n_raw);
//    for (i=0; i<n_raw; i++) in[i] = in1[i] + 
//                                    //in2[i] + 
//                                    //in3[i] + 
//                                    //in4[i] +
//                                    //in5[i] +
//                                    //in6[i] +
//                                    //in7[i] +
//                                    /*
//                                    in8[i] +
//                                    in9[i] +
//                                    in10[i] +
//                                    in11[i] +
//                                    in12[i] + 
//                                    in13[i] + 
//                                    in14[i] +
//                                    in15[i] +
//                                    in16[i] +
//                                    in17[i] +
//                                    in18[i] +
//                                    in19[i] +
//                                    */
//                                    in20[i];
//
//    decimate(in,out,dt_raw,dt,n_raw,n);
//
//    spctrm(in,psd_raw,m_raw,k,1);
//    for (i=0;i<m_raw;i++) psd_raw[i] *= 2.0*(double)m_raw*(double)dt_raw;
//    for (i=0;i<m_raw;i++) f_raw[i] = (i/(2.0*m_raw*dt_raw));
//    
//    ido_dpsdin = fopen("out/predec_psd.out","w");
//	if (ido_dpsdin == NULL)
//		perror("Error precec_psd.out");
//    ido_dfin = fopen("out/predec_f.out","w");
//	if (ido_dfin == NULL)
//		perror("Error precec_psd.out");
//    for ( i=0; i<m_raw; i++) fprintf(ido_dpsdin,"%f\n",psd_raw[i]);
//    for ( i=0; i<m_raw; i++) fprintf(ido_dfin,"%f\n",f_raw[i]);
//    fclose(ido_dpsdin);
//    fclose(ido_dfin);
//
//    spctrm(out,psd,m,k,1);
//    spctrm(outo,psdo,m,k,1);
//    for (i=0;i<m;i++) psd[i] *= 2.0*(double)m*(double)dt;
//    for (i=0;i<m;i++) psdo[i] *= 2.0*(double)m*(double)dt;
//    for (i=0;i<m;i++) f[i] = (i/(2.0*m*dt));
//
//    ido_dpsd = fopen("out/postdec_psd_tuck.out","w");
//	if (ido_dpsd == NULL)
//		perror("Error postdec_psd_tuck.out");
//    ido_dpsdo = fopen("out/postdec_psd.out","w");
//	if (ido_dpsdo == NULL)
//		perror("Error postdec_psd.out");
//    ido_df = fopen("out/postdec_f.out","w");
//	if (ido_df == NULL)
//		perror("Error postdec_f.out");
//    for ( i=0; i<m; i++) fprintf(ido_dpsd,"%f\n",psd[i]);
//    for ( i=0; i<m; i++) fprintf(ido_dpsdo,"%f\n",psdo[i]);
//    for ( i=0; i<m; i++) fprintf(ido_df,"%f\n",f[i]);
//    fclose(ido_dpsd);
//    fclose(ido_dpsdo);
//    fclose(ido_df);
//    
//    return r;
//}

int test_spectrum_cycle(long double period, long double amp, float dt, int m){ 

    int r=0,i,id,id_noise;
    int r_peak=0, r_amp=0, r_peak_noise=0, r_amp_noise=0;
    int k = 2;
    int n = 5*m;
    float rndm;
    float noise_level = .25;
    long double disp[n], disp_noise[n];
    long double f[m];
    long double psd[m], psd_noise[m];
    long double max=0.0, max_noise=0.0;
    long double msa_t=0.0, msa_f=0.0;
    long double msa_t_noise=0.0, msa_f_noise=0.0;

    periodic_func(disp,period,amp,dt,n);
    spctrm_psd(disp,psd,m,k);

    // peak psd test
    for (i=0; i<m; i++) f[i] = (i/(2.0*m*dt));
    for (i=0; i<m; i++) {
         if (psd[i] > max) {
             max = psd[i];
             id = i;
         }
    }
    if (fabsl(1/f[id]-period) > 0.3) r_peak = 1;
    if (r_peak)  {
        debug("                peak psd test failed",0);
        debug("                period         ",period);
        debug("                pk period      ",1.0/f[id]);
    }

    // mean squared amplitude test 
    for(i=0;i<m;i++) msa_f = msa_f + psd[i];
    for(i=0;i<n;i++) msa_t = msa_t + pow(fabsl(disp[i]),2);
    msa_t = msa_t/n;
    if (fabsl(msa_t-msa_f) > 0.1) r_amp = 1;
    if (r_amp)  {
        debug("                mean sq amplitude test failed",0);
        debug("                mean sq amp - time domain", msa_t);
        debug("                mean sq amp - freq domain", msa_f);
    }

    // generate noise
    rndm = rand()%100;
    for (i = 0; i<n; i++) disp_noise[i] = disp[i] + amp*noise_level*rndm/100.0;

    // spectrum
    spctrm_psd(disp_noise,psd_noise,m,k);

    // peak psd test with noise
    for (i=0; i<m; i++) {
         if (psd_noise[i] > max_noise) {
             max_noise = psd_noise[i];
             id_noise = i;
         }
    }
    if (fabsl(1/f[id_noise]-period) > 0.3) r_peak_noise = 1;
    if (r_peak_noise)  {
        debug("                peak psd noise test failed",0);
        debug("                period         ",period);
        debug("                pk period_noise",1.0/f[id_noise]);
    }

    // mean squared amplitude test 
    for(i=0;i<m;i++) msa_f_noise = msa_f_noise + psd_noise[i];
    for(i=0;i<n;i++) msa_t_noise = msa_t_noise + pow(fabsl(disp_noise[i]),2);
    msa_t_noise = msa_t_noise/n;
    if (fabsl(msa_t_noise-msa_f_noise) > 0.1) r_amp_noise = 1;
    if (r_amp_noise) {
        debug("                mean sq amplitude noise test failed",0);
        debug("                mean sq amp - time domain_noise", msa_t_noise);
        debug("                mean sq amp - freq domain_noise", msa_f_noise);
    }

    if (r_peak || r_amp || r_peak_noise || r_amp_noise != 0) r = 1;

    if ( (verbose_error && r == 1) || (verbose) )

    return r;
}

