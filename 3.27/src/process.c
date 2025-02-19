/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

// XXX

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "accel_calib.h"
#include "util.h"
#include "main.h"
#include "process_data.h"
#include "signal_conditioning.h"

void rspns(long double R[], long double f[], int n, long double f1, long double f2);
void deterministic_analysis(long double disp[],long double hz_max[],long double hcm_max[],long double htm_max[],long double tz_mean[], long double hz_mean[]);
void spctrm_psd(long double *disp, long double *psd, int mm, int kk);
void spctrm_imu(long double *disp, long double *psd, int mm, int kk);
int wave_direction(struct DATARAW *data_raw, struct RETDATA *return_data);
int wave_direction_FEM(struct DATARAW *data_raw, struct RETDATA *return_data);
int wave_direction_MEM(struct DATARAW *data_raw, struct RETDATA *return_data);
int wave_direction_MEM_raw(struct DATARAW *data_raw, struct RETDATA *return_data);
int quality_control_fft_trend(long double *data,long double *std);
long double identify_low_frequency_noise(struct RETDATA *return_data, long double *psd, long double *f, int n);

// ********************************************************************************
// ********************************************************************************
int ProcessData(
        struct DATARAW *data_raw,
	struct DATA2HZ *data,
	// XXX change this to a struct too ! combine data only when sending to Satellite, not here !!!
	struct RETDATA *return_data
  // What do we need to return...
) {

  logger("Start ProccesData",0.0);

  //////////////////////////////////////
  // Local variables
  //////////////////////////////////////
  int i, j, strt, st, ed;
  long double temp;
  long double disp_f1f2[n_spec]; 
  long double disp_f3f4[n_spec]; 
  long double disp_z[n_spec]; 
  long double disp_k[n_spec]; 
  long double R_t[n_accel-1]; // response function in the time domain
  long double psd_f1f2[m+1]; // Power spectral density
  long double psd_f3f4[m+1]; // Power spectral density
  long double psd_z[m+1]; // Power spectral density
  long double f[n_accel]; // frequency domain from raw acceleration
  long double ff[m+1], f_avg[n_psd]; // frequency domain after processing
  long double msa_t, msa_f; // mean squared amplitude
  long double moment_imu[n_mom];
  long double peak, Tp;
  long double heave[n_spec], roll[n_spec], pitch[n_spec];
  long double f2;

  
  ///////////////////////////////////////////////////////////////////
  // Extend number of acceleration samples to a power of 2  
  ///////////////////////////////////////////////////////////////////
  for (i=0;i<n_accel;i++) data->accel_zeros[i] = data->accel_z[i];
  for (i=n_accel;i<n_accelfft;i++) data->accel_zeros[i] = 0.0;
  
  ///////////////////////////////////////////////////////////////////
  // Spectral analysis based on calculated dominant frequency band 
  ///////////////////////////////////////////////////////////////////
  #ifdef ID_SPECT_BAND
  logger("Start identify spectral band",0.0);
  intgrt_press(data->accel_zeros,disp_f1f2,dt,n_accelfft,n_spec,rspns_f1,rspns_f2);
  spctrm_psd(disp_f1f2,psd_f1f2,m,k);
  spctrm_psd(disp_f1f2,psd_f1f2,m,k);
  // Normalise
  for (i=0;i<m+1;i++) psd_f1f2[i] *= 2.0*(long double)m*(long double)dt;///3686296.25;
  // Define bins
  for (i=0;i<m+1;i++) ff[i] = (i/(2.0*m*dt));
  return_data->f2 = identify_low_frequency_noise(return_data, psd_f1f2, ff, m+1);
  if (return_data->f2 >0.25) return_data->f2 = rspns_f4;
  if (return_data->f2 < rspns_f2) return_data->f2 = rspns_f4;
  intgrt_press(data->accel_zeros,disp_z,dt,n_accelfft,n_spec,return_data->f2-0.01,return_data->f2);
  logger("End identify spectral band",0.0);
  #else
  return_data->f2 = rspns_f4;
  intgrt_press(data->accel_zeros,disp_z,dt,n_accelfft,n_spec,rspns_f3,rspns_f4);
  #endif
  
  // Deterministic analysis
  return_data->hmod = 0.0;
  for(i=0;i<n_spec;i++) return_data->hmod += pow(disp_z[i],2);
  return_data->hmod /= (float)n_spec;
  return_data->hmod = 4.0*sqrt(return_data->hmod);
  deterministic_analysis(disp_z,return_data->hz_max,return_data->hcm_max,return_data->htm_max,return_data->tz_mean,return_data->hz_mean);  

  // Direction
  wave_direction_FEM_2hz_accel(data->accel_z,data->pitch,data->roll,return_data);

  // FFT using spectrm
  // ****** TO DEBUG: spectrm_psd required twice *******
  spctrm_psd(disp_z,psd_z,m,k);
  spctrm_psd(disp_z,psd_z,m,k);

  // Normalise
  for (i=0;i<m+1;i++) psd_z[i] *= 2.0*(long double)m*(long double)dt;///3686296.25;
  
  // Define bins
  for (i=0;i<m+1;i++) ff[i] = (i/(2.0*m*dt));

  // Tp 
  peak = 0.0;
  for (i=0;i<m+1;i++) {
       if (peak<=psd_z[i]) {
           peak = psd_z[i];
           return_data->tp = 1.0/ff[i];
       }
  }

  // Spectral moments 
  for (j=0;j < n_mom;j++){
         return_data->moment[j] = 0.0;
         // note that start from psd(1) to exclude dc component 
         for (i=1;i<m+1;i++) return_data->moment[j] +=(pow(ff[i],(j-2))*ff[1]*psd_z[i]); 
  }

  ///////////////////////////////////////////////////////////////////
  // Spectral analysis on full frequency band (rspns_f3 - rspns_f4)
  ///////////////////////////////////////////////////////////////////
  intgrt_press(data->accel_zeros,disp_f3f4,dt,n_accelfft,n_spec,rspns_f3,rspns_f4);
  spctrm_psd(disp_f3f4,psd_f3f4,m,k);
  spctrm_psd(disp_f3f4,psd_f3f4,m,k);

  // Normalise
  for (i=0;i<m+1;i++) psd_f3f4[i] *= 2.0*(long double)m*(long double)dt;///3686296.25;

  i=1; j=0;
  while ((ff[i]<0.51)&&(i<m)&&(j<n_psd)){
         if (ff[i] < 0.03) i++;
         else if (ff[i]<0.18) {
                return_data->psd[j] = (psd_f3f4[i]); 
                f_avg[j] = ff[i];
                j++;
                i+=1;
         }else if (ff[i]<0.25) {
                return_data->psd[j]= (psd_f3f4[i-1] + psd_f3f4[i] + psd_f3f4[i+1])/3.0;
                f_avg[j] = ff[i];
                j++;
                i+=3;
         }else if (ff[i]<0.3) {
                return_data->psd[j]= (psd_f3f4[i-2] + psd_f3f4[i-1] + psd_f3f4[i] + psd_f3f4[i+1] + psd_f3f4[i+2])/5.0;
                f_avg[j] = ff[i];
                j++;
                i+=5;
         }else if (ff[i]<0.5){
                return_data->psd[j]= (psd_f3f4[i-3] + psd_f3f4[i-2] + psd_f3f4[i-1] + psd_f3f4[i] + psd_f3f4[i+1] + psd_f3f4[i+2] + psd_f3f4[i+3])/7.0;
                f_avg[j] = ff[i];
                j++;
                i+=7;
         }
  }

  //////////////////////////////////////
  // Mean squared amplitude test 
  //////////////////////////////////////
  
  // frequency domain
  msa_f=0;
  for (i=0;i<m+1;i++) msa_f += psd_f3f4[i]; 
  msa_f /= (2.0*(long double)m*(long double)dt);
  // time domain
  msa_t= 0;
  for (i=0; i<n_spec; i++) msa_t += fabs(pow(disp_z[i],2)); 
  msa_t /= n_spec;
  // Check for equality - should be close to zero 
  return_data->qflg_tot_pow_imu = fabsl(msa_t-msa_f);

  //////////////////////////////////////
  // Print to output file 
  //////////////////////////////////////
  ido_o = fopen("out/processed.out","w");
  if (ido_o == NULL) perror("Error processed.out");
  for (i=0;i<2*k;i++) fprintf(ido_o,"%LF\n",return_data->hz_max[i]); 
  for (i=0;i<2*k;i++) fprintf(ido_o,"%LF\n",return_data->hcm_max[i]); 
  for (i=0;i<2*k;i++) fprintf(ido_o,"%LF\n",return_data->htm_max[i]); 
  for (i=0;i<2*k;i++) fprintf(ido_o,"%LF\n",return_data->tz_mean[i]); 
  fprintf(ido_o,"%LF\n",return_data->tp); 
  fprintf(ido_o,"%LF\n",return_data->hmod); 
  for (i=0;i<n_psd;i++) fprintf(ido_o,"%LF\n",return_data->psd[i]); 
  for (i=0;i<n_mom;i++) fprintf(ido_o,"%LF\n",return_data->moment[i]); 
  fprintf(ido_o,"%LF\n",return_data->direction*180.0/pi);
  fprintf(ido_o,"%LF\n",return_data->peak_direction*180.0/pi);
  fprintf(ido_o,"%LF\n",return_data->spread*180.0/pi);
  fprintf(ido_o,"%LF\n",return_data->ratio);
  fprintf(ido_o,"%LF\n",return_data->hs_dir);
  fprintf(ido_o,"%LF\n",return_data->a);
  fprintf(ido_o,"%LF\n",return_data->b);
  fprintf(ido_o,"%LF\n",return_data->nstd);
  fprintf(ido_o,"%LF\n",return_data->f2);
  fprintf(ido_o,"%LF\n",return_data->qflg_mean_removed_imu);
  fprintf(ido_o,"%d\n",return_data->qflg_kist);
  fprintf(ido_o,"%d\n",return_data->qflg_imu);
  fprintf(ido_o,"%d\n",return_data->qflg_pkist);
  fprintf(ido_o,"%d\n",return_data->qflg_accel);
  fprintf(ido_o,"%d\n",return_data->qflg_gyro);
  fprintf(ido_o,"%d\n",return_data->qflg_magno);
  fprintf(ido_o,"%d\n",return_data->qflg_head);
  fprintf(ido_o,"%LF\n",return_data->qflg_tot_pow_imu);
  fprintf(ido_o,"%LF\n",return_data->std_yaw);
  fprintf(ido_o,"%d\n",return_data->qflg_open_water);
  fprintf(ido_o,"%LF\n",return_data->hz_mean[0]);
  #ifdef SEND_DIRECTION
     // Find range for directional analysis
     for (i = 0; i<m+1; i++) {
          if (ff[i] > rspns_f7) {
              st = i;
              break;
          }
     }
     for (i = 0; i<m+1; i++) {
          if (ff[i] > rspns_f8) {
              ed = i;
              break;
          }
     }
     for (i=st;i<ed;i++) fprintf(ido_o,"%d\n",return_data->direction_full[i]); 
  #endif
  fclose(ido_o);

  //////////////////////////////////////
  // Print to command line 
  //////////////////////////////////////
  #ifdef COMMAND_LINE 
  for (i=0;i<2*k;i++) debug("   Hz_max", return_data->hz_max[i]); // Hz(max,k=1:4)
  for (i=0;i<2*k;i++) debug("   Hcm_max", return_data->hcm_max[i]); // Hcm(max,k=1:4)
  for (i=0;i<2*k;i++) debug("   Htm_max", return_data->htm_max[i]); // Htm(max,k=1:4)
  debug("   Hz_mean ",return_data->hz_mean[0]);
  for (i=0;i<2*k;i++) debug("   Tz_mean", return_data->tz_mean[i]); // tz(mean,k=1:4)
  debug("   Tp (full spectra)", return_data->tp);
  debug("   Hmo (deterministic)", return_data->hmod); // Hmo deterministic 
  debug("   Hmo (moments)", 4*sqrt(return_data->moment[2]));
  debug("   Hs (direction)", return_data->hs_dir);
  debug("   Mean wave direction", return_data->direction*180.0/pi); // convert to degrees True 
  debug("   Peak wave direction", return_data->peak_direction*180.0/pi); // convert to degrees True 
  debug("   Spread ", return_data->spread*180/pi);
  debug("   Ratio ", return_data->ratio);
  debug("   a", return_data->a); // noise slope 
  debug("   b", return_data->b); // noise intercept 
  debug("   nstd", return_data->nstd); // noise standard deviation 
  debug("   f2", return_data->f2); // response cut-off 
  debug("   Mean acceleration removed ", return_data->qflg_mean_removed_imu);
  debug("   Qflag kist ", return_data->qflg_kist);
  debug("   Qflag imu ", return_data->qflg_imu);
  debug("   Qflag percentage kist ", return_data->qflg_pkist);
  debug("   Qflag accel ", return_data->qflg_accel);
  debug("   Qflag gyro ",return_data->qflg_gyro);
  debug("   Qflag magno ",return_data->qflg_magno);
  debug("   Qflag head ",return_data->qflg_head);
  debug("   Total power check", return_data->qflg_tot_pow_imu);
  debug("   Standard deviation yaw ", return_data->std_yaw*180.0/pi);
  debug("   Open water ",return_data->qflg_open_water);
  #endif

  //////////////////////////////////////
  // Print variables to out/ for testing
  //////////////////////////////////////
  print2out(disp_z,n_spec,"out/disp_z.out",ido_tdz);
  print2out(ff,m+1,"out/f.out",ido_favg);
  print2out(f_avg,n_psd,"out/f_avg.out",ido_favg);
  print2out(psd_f1f2,m+1,"out/psdf1f2.out",ido_sz);
  print2out(psd_f3f4,m+1,"out/psdf3f4.out",ido_sz);
  print2out(psd_z,m+1,"out/psd_z.out",ido_sz);
  print2out(moment_imu,n_mom,"out/moment_imu.out",ido_momz);
  print2out(return_data->moment,n_mom,"out/moment_kist.out",ido_momk);

// Print power check to quality_control.out
  ido_oqc = fopen("out/quality_control.out","a");
  if (ido_oqc == NULL) perror("Error quality_control.out");
  fprintf(ido_oqc,"Total power check x = %LF\n",return_data->qflg_tot_pow_imu);
  fclose(ido_oqc);

  logger("End ProccesData",0.0);
  return 0;
} // end ProcessData

