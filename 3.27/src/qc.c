/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "util.h"
#include "main.h"
#include "signal_conditioning.h"
#include "process_data.h"

int quality_control_rpy(long double *data);

// ********************************************************************************
// ********************************************************************************

void quality_control(struct DATARAW *data_raw, struct RETDATA *return_data){

     logger("Start quality_control",0.0);
     ido_oqc = fopen("out/quality_control.out","w");
     if (ido_oqc == NULL) perror("Error quality_control.out");

     quality_control_accel(data_raw,return_data);
     quality_control_gyro(data_raw,return_data);
     quality_control_magno(data_raw,return_data);
     quality_control_rollpitch(data_raw,return_data);
     quality_control_heading(data_raw,return_data);

     if (return_data->qflg_accel > 80) return_data->qflg_imu = 1;
     if (return_data->qflg_rollpitch == 1 ) return_data->qflg_imu = 1;

     fclose(ido_oqc);
     logger("End quality_control",0.0);
     return;
}

// ********************************************************************************
// ********************************************************************************

void quality_control_accel(struct DATARAW *data_raw, struct RETDATA *return_data){

    logger("Start quality_control_accel",0.0);

    int i, r;
    int t1 = 0;
    int n_flat_k=0,n_flat_x=0, n_flat_y=0, n_flat_z=0; 
    int max_flat_k=0, max_flat_x=0, max_flat_y=0, max_flat_z=0;
    int n_spike_k=0, n_spike_x=0, n_spike_y=0, n_spike_z=0; 
    int max_spike_k=0, max_spike_x=0, max_spike_y=0, max_spike_z=0;
    int qflg_k=0,qflg_x=0,qflg_y=0,qflg_z=0;
    int temp_n, temp_max;

    // Flat
    qflg_k = flat(data_raw->accel_raw_kist,&n_flat_k,&max_flat_k,0.5,n_raw); 
    if (qflg_k == 1) return_data->qflg_kist = 1;
    patch(data_raw->accel_raw_kist,n_raw);
    qflg_x = flat(data_raw->accel_raw_x,&n_flat_x,&max_flat_x,0.5,n_raw); 
    patch(data_raw->accel_raw_x,n_raw);
    qflg_y = flat(data_raw->accel_raw_y,&n_flat_y,&max_flat_y,0.5,n_raw); 
    patch(data_raw->accel_raw_y,n_raw);
    qflg_z = flat(data_raw->accel_raw_z,&n_flat_z,&max_flat_z,0.5,n_raw); 
    patch(data_raw->accel_raw_z,n_raw);

    // Despike 
    // run despike on Kistler several times to remove all spikes
    for (i=0;i<10;i++){
         despike(data_raw->accel_raw_kist,&temp_n,&temp_max,n_raw,nstd_kist); 
         patch(data_raw->accel_raw_kist,n_raw);
         n_spike_k = n_spike_k + temp_n;
         max_spike_k = max_spike_k + temp_max;
    }
    despike(data_raw->accel_raw_x,&n_spike_x,&max_spike_x,n_raw,nstd_imu); 
    patch(data_raw->accel_raw_x,n_raw);
    despike(data_raw->accel_raw_y,&n_spike_y,&max_spike_y,n_raw,nstd_imu); 
    patch(data_raw->accel_raw_y,n_raw);
    despike(data_raw->accel_raw_z,&n_spike_z,&max_spike_z,n_raw,nstd_imu); 
    patch(data_raw->accel_raw_z,n_raw);

    // Test stats
    if (return_data->qflg_kist == 0) {
        return_data->qflg_kist = test_stats(data_raw->accel_raw_kist,n_raw);
    }
    r = test_stats(data_raw->accel_raw_x,n_raw);
    if (r == 1) return_data->qflg_accel = 100;
    r = test_stats(data_raw->accel_raw_y,n_raw);
    if (r == 1) return_data->qflg_accel = 100;
    r = test_stats(data_raw->accel_raw_z,n_raw);
    if (r == 1) return_data->qflg_accel = 100;
        
    // Return flags
    if (return_data->qflg_accel <100) {
        return_data->qflg_accel = (int)100*(n_flat_x + n_spike_x + n_flat_y + n_spike_y + n_flat_z + n_spike_z)/(n_raw*3);
    }
    return_data->qflg_pkist = (int)100*(n_flat_k + n_spike_k)/(n_raw);

    // Print  
    #ifdef PRINT_OUTPUT 
    fprintf(ido_oqc,"ACCEL QUALITY FLAGS\n");
    fprintf(ido_oqc,"Quality flag x = %d\n", qflg_x);
    fprintf(ido_oqc,"Quality flag y = %d\n", qflg_y);
    fprintf(ido_oqc,"Quality flag z = %d\n", qflg_z);
    fprintf(ido_oqc,"Number of unresponsive x = %d\n",n_flat_x);
    fprintf(ido_oqc,"Number of unresponsive y = %d\n",n_flat_y);
    fprintf(ido_oqc,"Number of unresponsive z = %d\n",n_flat_z);
    fprintf(ido_oqc,"Number of spikes x = %d\n",n_spike_x);///(long double)n_raw);
    fprintf(ido_oqc,"Number of spikes y = %d\n",n_spike_y);///(long double)n_raw);
    fprintf(ido_oqc,"Number of spikes z = %d\n",n_spike_z);///(long double)n_raw);
    #endif
    
    logger("End quality_control_accel",0.0);

    return ;
}

// ********************************************************************************
// ********************************************************************************

void quality_control_gyro(struct DATARAW *data_raw, struct RETDATA *return_data){

    logger("Start quality_control_gyro",0.0);

    int i, r;
    int t1 = 0;
    int n_flat_x, n_flat_y, n_flat_z; 
    int max_flat_x, max_flat_y, max_flat_z;
    int n_spike_x, n_spike_y, n_spike_z; 
    int max_spike_x, max_spike_y, max_spike_z;
    int qflg_x=0,qflg_y=0,qflg_z=0;

    // Flat
    qflg_x = flat(data_raw->gyro_raw_x,&n_flat_x,&max_flat_x,0.5,n_raw); 
    patch(data_raw->gyro_raw_x,n_raw);
    qflg_y = flat(data_raw->gyro_raw_y,&n_flat_y,&max_flat_y,0.5,n_raw); 
    patch(data_raw->gyro_raw_y,n_raw);
    qflg_z = flat(data_raw->gyro_raw_z,&n_flat_z,&max_flat_z,0.5,n_raw); 
    patch(data_raw->gyro_raw_z,n_raw);

    // Despike 
    despike(data_raw->gyro_raw_x,&n_spike_x,&max_spike_x,n_raw,nstd_gyro); 
    patch(data_raw->gyro_raw_x,n_raw);
    despike(data_raw->gyro_raw_y,&n_spike_y,&max_spike_y,n_raw,nstd_gyro); 
    patch(data_raw->gyro_raw_y,n_raw);
    despike(data_raw->gyro_raw_z,&n_spike_z,&max_spike_z,n_raw,nstd_gyro); 
    patch(data_raw->gyro_raw_z,n_raw);

    // Test stats
    r = test_stats(data_raw->gyro_raw_x,n_raw);
    r = test_stats(data_raw->gyro_raw_y,n_raw);
    r = test_stats(data_raw->gyro_raw_z,n_raw);

    // Return flags
    return_data->qflg_gyro = (int)100*(n_flat_x + n_spike_x + n_flat_y + n_spike_y + n_flat_z + n_spike_z)/(n_raw*3);

    // Print  
    #ifdef PRINT_OUTPUT 
    fprintf(ido_oqc,"GYRO QUALITY FLAGS\n");
    fprintf(ido_oqc,"Quality flag x = %d\n", qflg_x);
    fprintf(ido_oqc,"Quality flag y = %d\n", qflg_y);
    fprintf(ido_oqc,"Quality flag z = %d\n", qflg_z);
    fprintf(ido_oqc,"Number of unresponsive x = %d\n",n_flat_x);
    fprintf(ido_oqc,"Number of unresponsive y = %d\n",n_flat_y);
    fprintf(ido_oqc,"Number of unresponsive z = %d\n",n_flat_z);
    fprintf(ido_oqc,"Number of spikes x = %d\n",n_spike_x);///(long double)n_raw);
    fprintf(ido_oqc,"Number of spikes y = %d\n",n_spike_y);///(long double)n_raw);
    fprintf(ido_oqc,"Number of spikes z = %d\n",n_spike_z);///(long double)n_raw);
    #endif
    
    logger("End quality_control_gyro",0.0);

    return ;

}

// ********************************************************************************
// ********************************************************************************

void quality_control_magno(struct DATARAW *data_raw, struct RETDATA *return_data){

    logger("Start quality_control_magno",0.0);

    int i, r;
    int t1 = 0;
    int n_spike_x, n_spike_y, n_spike_z; 
    int max_spike_x, max_spike_y, max_spike_z;
    int qflg_x=0,qflg_y=0,qflg_z=0;

    // Despike 
    despike(data_raw->magno_raw_x,&n_spike_x,&max_spike_x,n_raw,nstd_magno); 
    patch(data_raw->magno_raw_x,n_raw);
    despike(data_raw->magno_raw_y,&n_spike_y,&max_spike_y,n_raw,nstd_magno); 
    patch(data_raw->magno_raw_y,n_raw);
    despike(data_raw->magno_raw_z,&n_spike_z,&max_spike_z,n_raw,nstd_magno); 
    patch(data_raw->magno_raw_z,n_raw);

    // Test stats
    r = test_stats(data_raw->magno_raw_x,n_raw);
    r = test_stats(data_raw->magno_raw_y,n_raw);
    r = test_stats(data_raw->magno_raw_z,n_raw);

    // Return flags
    return_data->qflg_magno = (int)100*(n_spike_x + n_spike_y + n_spike_z)/(n_raw*3);

    // Print  
    #ifdef PRINT_OUTPUT 
    fprintf(ido_oqc,"MAGNO QUALITY FLAGS\n");
    fprintf(ido_oqc,"Quality flag x = %d\n", qflg_x);
    fprintf(ido_oqc,"Quality flag y = %d\n", qflg_y);
    fprintf(ido_oqc,"Quality flag z = %d\n", qflg_z);
    fprintf(ido_oqc,"Number of spikes x = %d\n",n_spike_x);///(long double)n_raw);
    fprintf(ido_oqc,"Number of spikes y = %d\n",n_spike_y);///(long double)n_raw);
    fprintf(ido_oqc,"Number of spikes z = %d\n",n_spike_z);///(long double)n_raw);
    #endif
    
    logger("End quality_control_magno",0.0);

    return;
}

// ********************************************************************************
// ********************************************************************************

void quality_control_rollpitch(struct DATARAW *data_raw, struct RETDATA *return_data){

    logger("Start quality_control_rollpitch",0.0);

    int r;

    // Test stats
    r = test_stats(data_raw->roll,n_raw);
    if (r==1) return_data->qflg_rollpitch = 1;
    r = test_stats(data_raw->pitch,n_raw);
    if (r==1) return_data->qflg_rollpitch = 1;

    // Print  
    #ifdef PRINT_OUTPUT 
    fprintf(ido_oqc,"ROLL/PITCH QUALITY FLAG\n");
    fprintf(ido_oqc,"Quality flag = %d\n", return_data->qflg_rollpitch);
    #endif

    logger("End quality_control_rollpitch",0.0);

    return;
}

// ********************************************************************************
// ********************************************************************************

void quality_control_heading(struct DATARAW *data_raw, struct RETDATA *return_data){

    logger("Start quality_control_heading",0.0);

    int r;
    int n_flat, max_flat;

    // Flat
    r = flat(data_raw->gps_headings,&n_flat,&max_flat,.95,return_data->n_gps); 
    if (r==1) return_data->qflg_head = 1;

    // Test stats
    r = test_stats(data_raw->gps_headings,return_data->n_gps);
    if (r==1) return_data->qflg_head = 1;

    // Print  
    #ifdef PRINT_OUTPUT 
    fprintf(ido_oqc,"HEADING QUALITY FLAG\n");
    fprintf(ido_oqc,"Quality flag = %d\n", return_data->qflg_head);
    #endif

    logger("End quality_control_heading",0.0);

    return;
}

// ********************************************************************************
// ********************************************************************************

void despike(long double *data, int *qflg, int *maxt, int n, int nstd) {

       logger("Start despike",0.0);

       int i, j, spike_index[n];
       long double mean = 0, mean_sq = 0, std;
       int count;

       count = 0;
       *maxt = 0;
       *qflg = 0; 

       for (j=0;j<3;j++){
            for (i=0;i<n;i++) {
                 mean += data[i];      
                 mean_sq += pow(data[i],2);
            }
            mean /= (n-*qflg);
            mean_sq /= (n-*qflg);
            std = sqrt(mean_sq-(mean*mean));

            for (i=0;i<n;i++){
                 if (fabsl(data[i]-mean)>fabsl(nstd*std)){
                 //if (fabs(data[i]-mean)>fabs(3*std)){
                     data[i] = 0;
                     spike_index[*qflg] = i;
                     *qflg = *qflg + 1;
                     count++;
                     if (count > *maxt) *maxt = count;
                 }else{
                     count = 0;
                 }
            }
       }

       for (i=0;i<*qflg;i++) data[spike_index[i]] = 9999.0; 

       logger("End despike",0.0);

       return;
}

// ********************************************************************************
// ********************************************************************************

int flat(long double *data, int *total, int *max, double qflg_max, int n ) {

       int i,j=0,r=0;
       int flat_id[n];
       int flat_count = 0;
       int temp=1;

       *total = 0;
       *max = 0;

       // Define no. of flat spots 
       for (i=1;i<n;i++){
              if (data[i]==data[i-1]) {
                     //if (flat_count == 0) {
                     //       flat_id[j] = i-1;
                     //       *total = *total + 1;
                     //       flat_count++;
                     //       j++;
                     //}
                     flat_id[j] = i;
                     *total = *total + 1;
                     flat_count++;
                     j++;
                     if (flat_count > *max) *max = flat_count;
              }
              else{
                   flat_count = 0;
              }
       }
       // fill flat spots with 9999's
       for (i=0;i<*total;i++) data[flat_id[i]] = 9999.0;

       if (*total > n*qflg_max) r = 1;

       return(r);
}

// ********************************************************************************
// ********************************************************************************

int quality_control_fft(long double *data){

    logger("Start quality_control_fft",0.0);

    int i;
    int r=0;
    int temp1=0, temp2=0;

    demean(data, n_raw);
    flat(data,&temp1,&temp2,0.5,n_raw);
    patch(data,n_raw); 
    demean(data, n_raw);
    despike(data,&temp1,&temp2,n_raw,nstd_pos);
    patch(data,n_raw);
    demean(data, n_raw); // demean again ( influence from spikes)
    trend_removal(data,n_raw);
    despike(data,&temp1,&temp2,n_raw,nstd_pos); // remove spikes again just to be sure
    patch(data,n_raw); 
    demean(data, n_raw); 
    test_stats(data,n_raw);

    for (i=0;i<n_raw;i++) if (isnan(data[i])) r = 1;

    logger("End quality_control_fft",(float) r);
    return(r);
}

// ********************************************************************************
// ********************************************************************************

int quality_control_rpy(long double *data){

    int i;
    int r=0;
    int temp1,temp2;
    int nstd=6;

    despike(data,&temp1,&temp2,n_raw,nstd);
    patch(data,n_raw);

    test_stats(data,n_raw);

    return(r);
}

// ********************************************************************************
// ********************************************************************************

int test_stats(long double *data, int n){

       // Test basic stats
       // DEBUG: This code fails when data is close to 0
       
       logger("Start test_stats",0.0);

       int i;
       float r=0.0;
       long double max=0.0, min=0.0, mean=0.0, mean_sq=0.0, sd;

       for (i=0; i<n; i++){
            if (data[i] > max) max=data[i];
            if (data[i] < min) min=data[i];
            mean += data[i];
            mean_sq += pow(data[i],2);
       }
       mean /= n;
       mean_sq /= n;
       sd = sqrt(mean_sq-pow(mean,2));
       
       if (mean >= max) r=1.0;
       if (mean <= min) r=1.0;
       if (max <= min) r=1.0;
       if (min >= max) r=1.0;
       if (sd > (max-min)) r=1.0;

       logger("End test_stats",r);
       return(r);
}

// ********************************************************************************
// ********************************************************************************

int trend_removal(long double *data, int n){

       int r = 0;
       int i;

       long double kk=0.9988; 
       long double sn_n=0, sn_o=0;

       for (i=0; i<n; i++){
            sn_n = data[i] + kk*sn_o;
            data[i] = data[i] - (1-kk)*sn_n;
            sn_o = sn_n;
       }

       return r;
}

// ********************************************************************************
// ********************************************************************************

int patch(long double *data, int n) {

       logger("Start patch",0.0);

       int i, j=0, jj;
       int qflg=0;
       int seq = 0, mb=0, mxb=0;
       int good[n+1];
       int xlast, xnext;
       long double next, last, xdiff, ydiff, slope;

       // Define good data indices and 
       for (i=0;i<n;i++){
              if (data[i]==9999.0) {
                     mb++;
                     if (mb>qflg) qflg=mb;
              }
              else{
                   j++;
                   good[j] = i;
                   seq = 0;
                   mb = 0;
                   mxb = 0;
              }
       }
       // Linearly interpolate bad data
       if (j != 0){
         for (jj=0;jj<=good[1];jj++) {
                 data[jj] = data[good[1]];
         }
         for (jj=good[j];jj<n;jj++) data[jj] = data[good[j]];
         for (i=good[1]; i<good[j]; i++){
              if (data[i] == 9999.0){
                  for (jj=j; jj>=1; jj--){
                       if (good[jj] < i ){
                           last = data[good[jj]];
                           xlast = good[jj];
                           break;
                       }
                       else {
                         last = data[good[1]];
                         xlast = good[1];
                       }
                   }
                   for (jj=1; jj<=j; jj++) {
                        if (good[jj]>i) {
                            next = data[good[jj]];
                            xnext = good[jj];
                            break;
                        }
                        else {
                            next = data[good[j]];
                            xnext = good[j];
                        }
                   }
                   ydiff = next - last;
                   xdiff = (long double) (xnext - xlast);
                   if (xdiff!=0.0) slope = ydiff/xdiff;
                   if (xdiff==0.0) slope = 0.0;
                   data[i]=last+slope*((long double)(i-xlast));
              }
           }
       }

       logger("End patch",0.0);

       return(qflg);
}
// ********************************************************************************
// ********************************************************************************

long double demean(long double *data, int n)
{
//       logger("Start demean",0);

       int i;
       long double total=0.0, opmean;

       for (i=0; i<n; i++) total += data[i];
       opmean = total/(long double) n;
       for (i=0; i<n; i++) {
            data[i] -= opmean;
            if (isnan(data[i])) logger("ERROR: Generating NaN's",1.0);
       }
//       logger("End demean",0);
       return(opmean);
 }

// ********************************************************************************
// ********************************************************************************

