/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

// PURPOSE: Misc maths functions

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "util.h"
#include "process_data.h"

void realft(long double data[], unsigned long n, int isign);
long double demean(long double *data, int n);  
int trend_removal(long double *data, int n);
int linreg(int no, long double *x, long double *y, long double a, long double b, long double r); 

// ********************************************************************************
// ********************************************************************************

void rspns(long double *R, long double *f, int n, long double f1, long double f2) {

       logger("Start rspns",0.0);

       int i;
       long double cosine;
       FILE *ido_response;

       for (i=0; i<n; i++){
              if (f[i]<f1){
                     R[i] = 0;
              }
              else if (f1 <= f[i] && f[i] < f2 ){
                     cosine = cos(pi*(f[i]-f1)/(f2-f1));
                     R[i] = (0.5)*(1.0-cosine)*(-1.0/pow(2*pi*f[i],2.0));
              }
              else {
                     R[i] = -1.0/pow(2.0*pi*f[i],2.0);
              }
       }
       
       print2out(R,n,"out/response.out",ido_response);

       logger("End rspns",0.0);

       return;
       
} 

// ********************************************************************************
// ********************************************************************************

long double identify_low_frequency_noise(struct RETDATA *return_data, long double *psd, long double *f, int n) {

       logger("Start identify_low_frequency_noise",0.0);

       int i,stt,stt1, ett1,stt2, ett2, cnt=0;
       long double x[n], y[n];
       long double sumx = 0.0;                        /* sum of x                      */
       long double sumx2 = 0.0;                       /* sum of x**2                   */
       long double sumxy = 0.0;                       /* sum of x * y                  */
       long double sumy = 0.0;                        /* sum of y                      */
       long double sumy2 = 0.0;                       /* sum of y**2                   */
       long double denom;

       // Low frequency noise
       cnt=0;
//       for (i=0;i<n;i++){ 
//            if (f[i] > rspns_f2) break;
//       }
//       stt1 = i;
//       for (i=0;i<n;i++){ 
//            if (f[i] > rspns_f5) break;
//       }
//       ett1 = i;
       stt1 = 3; // 85.333333 s
       ett1 = 10; // 25.6 s
       for (i=stt1; i<ett1; i++){
            x[cnt] = log(f[i]);
            y[cnt] = log(psd[i]);
            cnt++;
       }

       // High frequency noise
//       for (i=0;i<n;i++){ 
//            if (f[i] > rspns_f6) break;
//       }
//       stt2 = i;
//       ett2 = stt2 + (ett1-stt1);
       stt2 = 80; // 3.2 s
       ett2 = 129; // 1.98 s
       if(ett2>n) ett2=n;
       for (i=stt2; i<ett2; i++){
            x[cnt] = log(f[i]);
            y[cnt] = log(psd[i]);
            cnt++;
       }

       // Least square regression (log log of noise is linear)
       for (i=0;i<cnt;i++)   { 
          sumx  += x[i];       
          sumx2 += x[i]*x[i];  
          sumxy += x[i]*y[i];
          sumy  += y[i];      
          sumy2 += y[i]*y[i]; 
       } 
       denom = ((cnt) * sumx2 - (sumx*sumx));
       return_data->a = ((cnt) * sumxy  -  sumx * sumy) / denom;
       return_data->b = (sumy * sumx2  -  sumx * sumxy) / denom;
       return_data->r = (sumxy - sumx * sumy / (cnt)) / sqrt((sumx2 - (sumx*sumx)/(cnt)) * (sumy2 - (sumy*sumy)/(cnt)));
       if ((return_data->r>-0.5)&(return_data->r<0.5)) return(-1.0);
       if (isnan(return_data->r)) return(-1.0);

       // Calculate standard deviation of noise
       return_data->nstd = 0.0;
       for (i =0;i<cnt;i++) {
               return_data->nstd += pow((y[i] - return_data->a*x[i]+return_data->b),2);
       }
       return_data->nstd /= cnt;

       // Find lowest frequency above noise
       for (i=0;i<n;i++){ 
            if (f[i] > rspns_f2) break;
       }
       stt=i;
       cnt=0;
       for (i=stt; i<n ;i++) {
            if (log(psd[i]) >= (return_data->a*(log(f[i]))+return_data->b + (noise_factor*log(return_data->nstd))) ){
                cnt = i; 
                break;
            }
       }

       logger("End identify_low_frequency_noise",0.0);

       return(f[cnt]);
} 

// ********************************************************************************
// ********************************************************************************

  int intgrt_press(long double *accel, long double *disp, float sr, int n_a, int n_s, long double f1, long double f2) {

       logger("Start intgrt_press",0.0);

       int i, j, strt;
       float r = 0.0;
       long double f[n_a];
       long double R_f[n_a]; // response function in the frequency domain
       long double R_cmplx[n_a]; // response function expressed in complex notation
       long double disp_temp[n_a];

       // intialize
       for (i=0;i<n_s;i++) disp[i] = 0.0;

       // Allocate f
       for (i=0; i<n_a; i++) {
         f[i] = (long double)i/((long double)n_a*(long double)sr);
       }

       // Define the response function (weight function)
//       rspns(R_f, f, n_a,0.05,0.08);
//       rspns(R_f, f, n_a,0.001,0.002);
       rspns(R_f, f, n_a,f1,f2);

       // Add complex numbers to response function to enable convolution with accel
       R_cmplx[0] = R_f[0];
       R_cmplx[1] = R_f[n_a/2];
       for (i=2,j=1; i<n_a-1; i+=2,j++){
         if (j>=n_a) {
             fprintf(stderr,"ERROR: out of bounds in intgrt_press. Exiting.\n");
             exit(0);
         }
         R_cmplx[i] = R_f[j];
         R_cmplx[i+1] = R_f[j]; 
       }

       // Temporary allocation of disp and FFT acceleration
       FILE *ido_dt;
       for (i=0; i<n_a; i++) disp_temp[i] = accel[i]; 
       demean(disp_temp,n_a);
       realft(disp_temp,n_a,1);

       // Integrate 
       for(i=0;i<n_a;i++) {
               disp_temp[i] = disp_temp[i]*R_cmplx[i]; 
       }

       // High Pass Filter
       //HPF(disp_temp,f,n_a);
       
       // Reverse FFT
       realft(disp_temp,n_a,-1);
       // Normalise
       for (i=0; i<n_a; i++) disp_temp[i] *= 2.0/n_a; 
       print2out(disp_temp,n_a,"out/disp_full.out",ido_dt);

       // Remove noise at each end of the record
       strt = (n_a-n_s)/2;
       for (i=0; i<n_s; i++){
            if (strt+i>=n_a) {
                fprintf(stderr,"ERROR: out of bounds in intgrt_press. Exiting.\n");
                exit(0);
            }
            disp[i] = disp_temp[strt + i];
       }

       // Remove mean
       demean(disp,n_s);

       logger("End intgrt_press",0.0);
       return(0);

} // end intgrt_press

// ********************************************************************************
// ********************************************************************************

  long double intgrt_time_domain(long double *accel, long double *disp, float sr, int n_a, long double f1, long double f2) {

       logger("Start intgrt_time_domain",0.0);

       int i, j, strt;

       long double f[n_a];
       long double R_f[n_a]; // response function in the frequency domain
       long double R_cmplx[n_a]; // response function expressed in complex notation
       long double disp_temp[n_a];

       // Allocate f
       for (i=0; i<n_a; i++) {
         f[i] = (long double)i/((long double)n_a*(long double)sr);
       }

       // Define the response function (weight function)
       rspns(R_f, f, n_a,f1,f2);


       // Add complex numbers to response function to enable convolution with accel
       R_cmplx[0] = R_f[0];
       R_cmplx[1] = R_f[n_a-1];
       for (i=2,j=1; i<n_a-1; i+=2,j++){
         R_cmplx[i] = R_f[j];
         R_cmplx[i+1] = R_f[j]; 
       }

       // Temporary allocation of disp and FFT acceleration
       for (i=0; i<n_a; i++) disp_temp[i] = accel[i]; 
       demean(disp_temp,n_a);
       realft(disp_temp,n_a,1);

       // Integrate and high pass filter 
       for(i=0;i<n_a;i++) disp_temp[i] *= R_cmplx[i]; 
       realft(disp_temp,n_a,-1);
       for (i=0; i<n_a; i++) disp_temp[i] *= 2.0/n_a; 

       // Remove noise at each end of the record
       strt = (n_a-n_spec)/2;
       for (i=0; i<n_spec; i++){
              disp[i] = disp_temp[strt + i];
       }

       // Remove mean
       demean(disp,n_spec);

       logger("End intgrt_time_domain",0.0);

       return(0);

} // end intgrt_press

// ********************************************************************************
// ********************************************************************************

void deterministic_analysis(long double disp[],long double hz_max[], long double hcm_max[],long double htm_max[],long double tz_mean[], long double hz_mean[]){

     logger("Start deterministic_analysis",0.0);

     int i,j,count,zux_id,count_all,ok;
     long double dp[2*m]; 
     long double hz[2*k][n_spec/(2*k)],hcm[2*k][n_spec/(2*k)], htm[2*k][n_spec/(2*k)], tz[2*k][n_spec/(2*k)];
     long double max=0.0;

     // initialize 
     for (i=0;i<2*m;i++) dp[i] = 0.0;
     for (i=0;i<2*k;i++){
          hz_max[i] = 0.0;
          hcm_max[i] = 0.0;
          htm_max[i] = 0.0;
          tz_mean[i] = 0.0;
     }
     hz_mean[0] = 0.0;

     // find max displacement
     for (i=0;i<n_spec;i++) if (max < disp[i]) max=disp[i];

     count_all = 0;
     for (j=0,count=0; j<2*k; j++, count+=m) {
          zux_id = -1;
          ok=0;
          for (i=0;i<2*m;i++) {
               if (count+i>=n_spec) {
                   fprintf(stderr,"ERROR: out of bounds in deterministic_analysis. Exiting.\n");
                   exit(0);
               }
               dp[i] = disp[count+i];
               demean(dp,i);
               trend_removal(dp,2*m);
               demean(dp,i);
               if ( (i>0) && (((max >= 0.1)&&(dp[i]>0.005)&&(dp[i-1]<0.0)) || ((max <= 0.1)&&(dp[i]>0.00005)&&(dp[i-1]<0.0))) ) {
                  if (ok==1) {
                      hz_mean[0] += hz[j][zux_id];
                      count_all += 1;
                  }

                  zux_id++;
                  if (zux_id>=n_spec/(2*k)) {
                      fprintf(stderr,"ERROR: out of bounds in deterministic_analysis. Exiting.\n");
                      exit(0);
                  }
                  hcm[j][zux_id] = 0.0;
                  htm[j][zux_id] = 0.0;
                  hz[j][zux_id] = 0.0;
                  tz[j][zux_id] = i;
                  if (zux_id>0) {
                      tz_mean[j] += tz[j][zux_id] - tz[j][zux_id-1]; 
                  }
                  ok=1;
               }
               if (zux_id>-1) {
                   if (zux_id>=n_spec/(2*k)) {
                       fprintf(stderr,"ERROR: out of bounds in deterministic_analysis. Exiting.\n");
                       exit(0);
                   }
                   if ( hcm[j][zux_id] < dp[i] ) {
                        hcm[j][zux_id] = dp[i];
                   }
                   if ( htm[j][zux_id] > dp[i] ) {
                        htm[j][zux_id] = dp[i];
                   }
                   hz[j][zux_id] = hcm[j][zux_id] - htm[j][zux_id];
                   if (hcm_max[j] < hcm[j][zux_id]) {
                       hcm_max[j] = hcm[j][zux_id]; 
                   }
                   if (htm_max[j] > htm[j][zux_id]) {
                       htm_max[j] = htm[j][zux_id]; 
                   }
                   if (hz_max[j] < hz[j][zux_id]) {
                       hz_max[j] = hz[j][zux_id]; 
                   }
               }
          }
          if (zux_id>0) {
              tz_mean[j] /= zux_id;
              tz_mean[j] *= dt;
          }else{
              fprintf(stderr,"WARNING: No zero upcrossings.\n");
          }
     }
     hz_mean[0] /= count_all;

     logger("End deterministic_analysis",0.0);
     return;
}

// ********************************************************************************
// ********************************************************************************


