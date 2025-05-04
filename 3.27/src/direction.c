/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

// PURPOSE: Directional analysis 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "util.h"
#include "main.h"
#include "process_data.h"
#include "signal_conditioning.h"

void realft(long double data[], unsigned long n, int isign);

// ********************************************************************************
// ********************************************************************************

int north_east(struct DATARAW *data_raw, struct RETDATA *return_data){

    logger("Start north_east",0);

    int r=0, i = 0;
    double x,y,z,x_o,y_o;
    long double roll[n_raw],pitch[n_raw],yaw[n_raw];
    

    // Define Yaw
    #ifdef ARCTIC
    for (i=0; i<n_raw; i++) {
         yaw[i] = data_raw->yaw_arduino + data_raw->yaw[i]; 
         roll[i] = (cos(yaw[i])*tan(data_raw->pitch[i]+data_raw->pitch_arduino)) - (cos(yaw[i])*tan(data_raw->roll[i]+data_raw->roll_arduino)/cos(data_raw->pitch[i]+data_raw->pitch_arduino));
         pitch[i] = (cos(yaw[i])*tan(data_raw->pitch[i]+data_raw->pitch_arduino)) + ((sin(yaw[i])*tan(data_raw->roll[i]+data_raw->roll_arduino))/cos(data_raw->pitch[i]+data_raw->pitch_arduino));
         data_raw->roll[i] = roll[i];
         data_raw->pitch[i] = pitch[i];
         if ((isnan(data_raw->pitch[i]))||(isnan(data_raw->roll[i]))) r = 1;
    }
    #elif WAVETANK
       for (i=0; i<n_raw; i++) yaw[i] = 0; 
       return_data->qflg_head = 4;
    #else
       if (return_data->qflg_head == 0 ) {
           logger("GPS heading data used at 8 Hz",0.0);
           for (i=0; i<n_raw; i++) yaw[i] = data_raw->gps_headings_8hz[i]; 
       } else if (return_data->qflg_open_water == 1) {
           logger("Magnetometer yaw at each time step used for heading",0.0);
           for (i=0; i<n_raw; i++) yaw[i] = data_raw->yaw[i]; 
           return_data->qflg_head = 2;

       } else {
           if (data_raw->gps_heading > -9000) {
             logger("Instantaneous GPS heading used for heading",0.0);
             for (i=0; i<n_raw; i++) yaw[i] = data_raw->gps_heading;
             return_data->qflg_head = 1;
           } else {
             logger("Magnetometer mean yaw used for heading",0.0);
             for (i=0; i<n_raw; i++) yaw[i] = return_data->yaw_mean; 
             return_data->qflg_head = 3;
           }
       }
               
       // Define roll and pitch relative to true north
       for (i=0; i<n_raw; i++) {
            roll[i] = (cos(yaw[i])*tan(data_raw->pitch[i])) - (cos(yaw[i])*tan(data_raw->roll[i])/cos(data_raw->pitch[i]));
            pitch[i] = (cos(yaw[i])*tan(data_raw->pitch[i])) + ((sin(yaw[i])*tan(data_raw->roll[i]))/cos(data_raw->pitch[i]));
            data_raw->roll[i] = roll[i];
            data_raw->pitch[i] = pitch[i];
            if ((isnan(data_raw->pitch[i]))||(isnan(data_raw->roll[i]))) r = 1;
       }
    #endif

    logger("End north_east",r);
    return r;
}

/******************************************************************************
 * Function: wave_direction_FEM_2hz_accel
 * Purpose: Estimates wave direction using FEM (First-order Extended Maximum 
 *          likelihood Method) from 2Hz accelerometer and motion sensor data
 * 
 * Parameters:
 *   accel[]       - Vertical acceleration time series
 *   pitch[]       - Pitch motion time series
 *   roll[]        - Roll motion time series
 *   return_data   - Struct containing output parameters including:
 *                   - direction_full[] (Full directional spectrum)
 *                   - peak_direction   (Direction at spectral peak)
 *                   - direction        (Mean direction)
 *                   - spread          (Directional spread)
 *                   - ratio           (Directional quality parameter)
 *                   - hs_dir          (Significant wave height)
 * 
 * Returns:
 *   0 on success
 * 
 * Notes:
 *   - Uses cross-spectral analysis between vertical and horizontal motions
 *   - Frequency range defined by rspns_f7 to rspns_f8
 *   - Outputs directional parameters in degrees (0° = waves toward East)
 *   - Implements circular statistics for directional averaging
 *   - Includes spectral analysis using Welch's method
 *   
 * References:
 *   - Extended Maximum Likelihood Method for wave direction analysis
 *   - Circular statistics for directional spread calculation
 * 
 * Output Files:
 *   - roll_spectra.out  - Roll motion spectra
 *   - pitch_spectra.out - Pitch motion spectra
 *   - accel_spectra.out - Acceleration spectra
 *   - direction_full.out - Full directional spectrum
 *   - spread.out        - Directional spread
 * 
 * Author: [Your Name]
 * Date: [Date]
 *****************************************************************************/
int wave_direction_FEM_2hz_accel(long double *accel, long double *pitch, long double *roll, struct RETDATA *return_data){

       logger("Start wave_direction_FEM_2hz_accel",0);

       int i,j,peak_id;
       int w=m, ww=2*w, kk=2;
       int st, ed;
       int send_dir[w+1];
       long double df = 1/(ww*dt);
       long double c12[w+1], q12[w+1];
       long double c13[w+1], q13[w+1];
       long double c11[w+1], c22[w+1], c33[w+1];
       long double kd[w+1];
       long double fft_accel[ww], fft_ns[ww], fft_ew[ww];
       long double a1[w+1], b1[w+1];
       long double a2[w+1], b2[w+1];
       long double a3[w+1], b3[w+1];
       long double alpha12[w+1], alpha13[w+1];
       long double aa1[w+1], bb1[w+1], dir[w+1], sprd[w+1], ui[w+1];
       long double aa1s[w+1], bb1s[w+1];
       long double peak = 0, omega, mean_dir;
       long double Hs, c11_sum, dir_mean, ui_mean=0;
       long double ff[w+1];

       // Define ff
       for (i=0;i<w+1;i++) ff[i] = (i/(2.0*m*dt));

       // Initialize variables
       for (i=0;i<w+1;i++){
            c11[i] = 0.0;
            c22[i] = 0.0;
            c33[i] = 0.0;
            c12[i] = 0.0;
            q12[i] = 0.0;
            c13[i] = 0.0;
            q13[i] = 0.0;
            a1[i] = 0.0;
            a2[i] = 0.0;
            a3[i] = 0.0;
            b1[i] = 0.0;
            b2[i] = 0.0;
            b3[i] = 0.0;
            alpha12[i] = 0.0;
            alpha13[i] = 0.0;
            aa1[i] = 0.0;
            bb1[i] = 0.0;
            dir[i] = 0.0;
            sprd[i] = 0.0;
            send_dir[i] = 0;
       }
       for (i=0;i<ww;i++){
            fft_accel[i] = 0.0;
            fft_ns[i] = 0.0;
            fft_ew[i] = 0.0;
       }
       spctrm(roll,fft_ew,w,kk,n_spec,(float)dt);
       spctrm(pitch,fft_ns,w,kk,n_spec,(float)dt);
       spctrm(accel,fft_accel,w,kk,n_spec,(float)dt);
       print2out(fft_ew,ww,"out/roll_spectra.out",ido_rs);
       print2out(fft_ns,ww,"out/pitch_spectra.out",ido_ps);
       print2out(fft_accel,ww,"out/accel_spectra.out",ido_hs);

       // auto / cross correlations
       for (i=0,j=0; i<ww; i+=2,j++){
            if (j>=w+1) {
                fprintf(stderr,"ERROR: out of bounds in wave_direction_FEM_2hz. Exiting.\n");
                exit(0);
            }
            a1[j] = fft_accel[i];
            a2[j] = fft_ns[i];
            a3[j] = fft_ew[i]; 
       }
       a1[w] = fft_accel[ww-1];
       a2[w] = fft_ns[ww-1];
       a3[w] = fft_ew[ww-1];

       b1[0] = 0;
       b1[1] = 0;
       b2[0] = 0;
       b2[1] = 0;
       b3[0] = 0;
       b3[1] = 0;
       for (i=3,j=2; i<ww; i+=2,j++){
            if (j>=w+1) {
                fprintf(stderr,"ERROR: out of bounds in wave_direction_FEM_2hz. Exiting.\n");
                exit(0);
            }
            b1[j] = fft_accel[i];
            b2[j] = fft_ns[i];
            b3[j] = fft_ew[i]; 
       }
       b1[w] = 0;
       b2[w] = 0;
       b3[w] = 0;

       // Find range for directional analysis
       for (i = 0; i<w+1; i++) {
            if (ff[i] > rspns_f7) {
                st = i;
                break;
            }
       }
       for (i = 0; i<w+1; i++) {
            if (ff[i] > rspns_f8) {
                ed = i;
                break;
            }
       }

       c11_sum = 0;
       dir_mean = 0;
       Hs = 0;
       for (i = 1; i<m+1; i++){ 
            omega = (2.0*pi*ff[i])*(2.0*pi*ff[i]);
            c12[i] = a1[i]*a2[i]/omega + b1[i]*b2[i]/omega;
            q12[i] = a1[i]*b2[i]/omega - b1[i]*a2[i]/omega;
            c13[i] = a1[i]*a3[i]/omega + b1[i]*b3[i]/omega;
            q13[i] = a1[i]*b3[i]/omega - b1[i]*a3[i]/omega;
            c11[i] = a1[i]*a1[i]/(omega*omega) + b1[i]*b1[i]/(omega*omega);
            c22[i] = (a2[i]*a2[i] + b2[i]*b2[i]);
            c33[i] = (a3[i]*a3[i] + b3[i]*b3[i]);
            alpha12[i] = atan2(c12[i],q12[i]);
            alpha13[i] = atan2(c13[i],q13[i]);
            aa1[i] = (c12[i]*sin(alpha12[i]) + q12[i]*cos(alpha12[i]))/sqrt(c11[i]*(c22[i]+c33[i]));
            bb1[i] = (c13[i]*sin(alpha13[i]) + q13[i]*cos(alpha13[i]))/sqrt(c11[i]*(c22[i]+c33[i]));
            kd[i] = sqrt((c22[i]+c33[i])/c11[i]);
            aa1s[i] = q12[i]/(kd[i]*c11[i]);
            bb1s[i] = q13[i]/(kd[i]*c11[i]);
            // note that 0 deg is for waves headed towards positive x (EAST, right hand system)
            dir[i] = atan2(bb1[i],aa1[i]);
            ui[i] = sqrt(aa1s[i]*aa1s[i] + bb1s[i]*bb1s[i]);
            sprd[i] = sqrt(2.0*(1.0-ui[i])); //circular spread (Degrees) from Doble
            return_data->direction_full[i] = round(dir[i]*1800.0/pi);
            if ((i >= st) && (i < ed)) {
                 c11_sum += c11[i];
                 dir_mean += dir[i];
                 ui_mean += ui[i];
                 if (peak<=c11[i]) {
                     peak = c11[i];
                     peak_id = i;
                 }
            }
       }
       return_data->peak_direction = dir[peak_id];
       return_data->direction = dir_mean/(ed-st);
       return_data->spread = sprd[peak_id];
       return_data->ratio = ui_mean/(ed-st);
       //return_data->ratio =sqrt(c11[peak_id]/(c22[peak_id]/kd[peak_id] + c33[peak_id]/kd[peak_id]));

       // Significant wave height
       return_data->hs_dir = 4.0*sqrt(c11_sum);

       print2out(c11,w+1,"out/c11.out",ido_od);
       print2out(dir,w+1,"out/direction_full.out",ido_od);
       print2out(sprd,w+1,"out/spread.out",ido_od);
       print2out(aa1,w+1,"out/aa1.out",ido_od);
       print2out(bb1,w+1,"out/bb1.out",ido_od);

       logger("End wave_direction_FEM_2hz",0);
       return(0);
}

// ********************************************************************************
// ********************************************************************************

int wave_direction_FEM(struct DATARAW *data_raw, struct RETDATA *return_data){

       logger("Start wave_direction_FEM",0);

       int i,j,peak_id;
       int w=2048, ww=2*w, kk=2;
       long double df = 1/(ww*dt_raw);
       long double c12[w+1], q12[w+1];
       long double c13[w+1], q13[w+1];
       long double c11[w+1], c22[w+1], c33[w+1];
       long double fft_accel[ww], fft_ns[ww], fft_ew[ww];
       long double a1[w+1], b1[w+1];
       long double a2[w+1], b2[w+1];
       long double a3[w+1], b3[w+1];
       long double alpha12m, alpha13m;
       long double aa1m, bb1m;
       long double alpha12[w+1], alpha13[w+1];
       long double aa1[w+1], bb1[w+1], dir[w+1], spread[w+1];
       long double peak = 0, omega, mean_dir;
       long double Hs, c11_sum, dir_mean;

       spctrm(data_raw->roll,fft_ew,w,kk,n_raw,dt_raw);
       spctrm(data_raw->pitch,fft_ns,w,kk,n_raw,dt_raw);
       spctrm(data_raw->accel_vert,fft_accel,w,kk,n_raw,dt_raw);

       // auto / cross correlations
       for (i=0,j=0; i<ww; i+=2,j++){
            if (j>=w+1) {
                fprintf(stderr,"ERROR: out of bounds in wave_direction_FEM. Exiting.\n");
                exit(0);
            }
            a1[j] = fft_accel[i];
            a2[j] = fft_ns[i];
            a3[j] = fft_ew[i]; 
       }
       a1[w] = fft_accel[ww-1];
       a2[w] = fft_ns[ww-1];
       a3[w] = fft_ew[ww-1];

       b1[0] = 0;
       b1[1] = 0;
       b2[0] = 0;
       b2[1] = 0;
       b3[0] = 0;
       b3[1] = 0;
       for (i=3,j=2; i<ww; i+=2,j++){
            if (j>=w+1) {
                fprintf(stderr,"ERROR: out of bounds in wave_direction_FEM. Exiting.\n");
                exit(0);
            }
            b3[j] = fft_ew[i]; 
            b2[j] = fft_ns[i];
            b1[j] = fft_accel[i];
       }
       b1[w] = 0;
       b2[w] = 0;
       b3[w] = 0;

       dir[0] = 0; //temporary nan fix
       c11_sum = 0;
       dir_mean = 0;
       for (i = 0; i<w+1; i++){
            omega = 2.0*pi*8/(ww);
            // roll * conjugate(heave)
            //Czx[i] = roll_real[i]*heave_real[i] + roll_complex[i]*heave_complex[i]; // real
            c12[i] = (a1[i]*a2[i] + b1[i]*b2[i]);///(-omega*omega);
            //Qzx[i] = heave_real[i]*roll_complex[i] - roll_real[i]*heave_complex[i]; // imaginary
            q12[i] = (a1[i]*b2[i] - b1[i]*a2[i]);///(-omega*omega);
            // pitch * conjugate(heave)
            //Czy[i] = pitch_real[i]*heave_real[i] + pitch_complex[i]*heave_complex[i]; // real
            c13[i] = (a1[i]*a3[i] + b1[i]*b3[i]);///(-omega*omega);
            //Qzy[i] = heave_real[i]*pitch_complex[i] - pitch_real[i]*heave_complex[i]; // imaginary
            q13[i] = (a1[i]*b3[i] - b1[i]*a3[i]);///(-omega*omega);

            c11[i] = (a1[i]*a1[i] + b1[i]*b1[i]);///(omega*omega*omega*omega);
            // Heave * conjugate(heave/)
            //Czz[i] = heave_real[i]*heave_real[i] + heave_complex[i]*heave_complex[i];
            c22[i] = a2[i]*a2[i] + b2[i]*b2[i];
            // Roll * conjugate(roll)
            //Cxx[i] = roll_real[i]*roll_real[i] + roll_complex[i]*roll_complex[i];
            c33[i] = a3[i]*a3[i] + b3[i]*b3[i];
            // Pitch * conjugate(pitch)
            //Cyy[i] = pitch_real[i]*pitch_real[i] + pitch_complex[i]*pitch_complex[i];
            alpha12[i] = atan2(c12[i],q12[i]);
            alpha13[i] = atan2(c13[i],q13[i]);
            aa1[i] = (c12[i]*sin(alpha12[i]) + q12[i]*cos(alpha12[i]))/sqrt(c11[i]*(c22[i]+c33[i]));
            bb1[i] = (c13[i]*sin(alpha13[i]) + q13[i]*cos(alpha13[i]))/sqrt(c11[i]*(c22[i]+c33[i]));
            dir[i] = atan2(bb1[i],aa1[i]);
            spread[i] = sqrt(2.0-2.0*sqrt(aa1[i]*aa1[i] + bb1[i]*bb1[i]));
            if (peak<=c11[i]) {
                peak = c11[i];
                peak_id = i;
            }
            c11_sum += c11[i];
            dir_mean += dir[i];
       }
       return_data->peak_direction = dir[peak_id];
       return_data->direction = dir_mean/(w+1);
       return_data->spread = spread[peak_id];
       return_data->ratio =sqrt(c11[peak_id]/(c22[peak_id]+c33[peak_id])); 

       // check ratios
       logger("peak_id",peak_id);
       for (i = 0; i<m+1; i++){ 
            logger("ratio",(c22[i]+c33[i])/c11[i]);
       } 
              
       debug("p",1.0/(peak_id*8.0/ww));
       // Significant wave height
       Hs = 4.0*sqrt(c11_sum*8.0/(ww));
       debug("   Hs(direction) ",Hs);

       print2out(c11,w+1,"out/c11.out",ido_od);
       print2out(dir,w+1,"out/direction.out",ido_od);
       print2out(spread,w+1,"out/spread.out",ido_od);

       logger("End wave_direction_FEM",0);
       return(0);
}

// ********************************************************************************
// ********************************************************************************

int wave_direction_MEM_raw(struct DATARAW *data_raw, struct RETDATA *return_data){
        
       int i,j,peak_id;
       int w=2048, ww=2*w, kk=3;
       long double Hs, c11_sum;
       long double x,y,z,x_o,y_o;
       long double temp, sum, alpha, e1_real, e1_imag, e2_real, e2_imag, y_real, y_imag;
       long double fft_roll[ww], fft_pitch[ww], fft_heave[ww];
       long double roll_real[w+1], pitch_real[w+1], heave_real[w+1];
       long double roll_complex[w+1], pitch_complex[w+1], heave_complex[w+1];
       long double Cxx[w+1], Cyy[w+1], Czz[w+1];
       long double Cxy[w+1], Czx[w+1], Czy[w+1];
       long double RP_complex[w+1], Qzx[w+1], Qzy[w+1];
       long double kd[w+1], a1[w+1], a2[w+1], b1[w+1], b2[w+1];
       long double c1_real[w+1], c1_imag[w+1], c2_real[w+1], c2_imag[w+1]; 
       long double p1_real[w+1], p1_imag[w+1], p2_real[w+1], p2_imag[w+1];
       long double x1_real[w+1], x1_imag[w+1], y1[w+1];
       long double dir[w+1], dir_mean;
       long double win[ww];
       long double peak, omega;

       spctrm(data_raw->roll,fft_roll,w,kk,n_raw,dt_raw);
       spctrm(data_raw->pitch,fft_pitch,w,kk,n_raw,dt_raw);
       spctrm(data_raw->accel_vert,fft_heave,w,kk,n_raw,dt_raw);

       // auto / cross correlations
       for (i=0,j=0; i<ww; i+=2,j++){
            roll_real[j] = fft_roll[i]; 
            pitch_real[j] = fft_pitch[i];
            heave_real[j] = fft_heave[i];
       }
       roll_real[w] = 0;
       pitch_real[w] = 0;
       heave_real[w] = 0;

       roll_complex[0] = 0;
       roll_complex[1] = 0;
       pitch_complex[0] = 0;
       pitch_complex[1] = 0;
       heave_complex[0] = 0;
       heave_complex[1] = 0;
       for (i=3,j=2; i<ww; i+=2,j++){
            roll_complex[j] = fft_roll[i]; 
            pitch_complex[j] = fft_pitch[i];
            heave_complex[j] = fft_heave[i];
       }
       roll_complex[w] = 0;
       pitch_complex[w] = 0;
       heave_complex[w] = 0;

       // Kuik et al.
       peak = 0;
       for (i=0;i<w+1;i++){
            omega = 2.0*pi*i*8.0/ww;
            // Roll * conjugate(roll)
            Cxx[i] = (roll_real[i]*roll_real[i] + roll_complex[i]*roll_complex[i])/(omega*omega*omega*omega);
            // Pitch * conjugate(pitch)
            Cyy[i] = pitch_real[i]*pitch_real[i] + pitch_complex[i]*pitch_complex[i];
            // Heave * conjugate(heave/)
            Czz[i] = heave_real[i]*heave_real[i] + heave_complex[i]*heave_complex[i];
            // roll * conjugate(pitch)
            Cxy[i] = roll_real[i]*pitch_real[i] + roll_complex[i]*pitch_complex[i]/(-omega*omega);
            // roll * conjugate(heave)
            Czx[i] = roll_real[i]*heave_real[i] + roll_complex[i]*heave_complex[i]/(-omega*omega); // real
            Qzx[i] = heave_real[i]*roll_complex[i] - roll_real[i]*heave_complex[i]/(-omega*omega); // imaginary
            // pitch * conjugate(heave)
            Czy[i] = pitch_real[i]*heave_real[i] + pitch_complex[i]*heave_complex[i]; // real
            Qzy[i] = heave_real[i]*pitch_complex[i] - pitch_real[i]*heave_complex[i]; // imaginary
            if (peak<=Czz[i]) {
                peak = Czz[i];
                peak_id = i;
            }
            c11_sum += Cxx[i];
       }

       // Kd
       for (i=0;i<w+1;i++){
            // Kuik method
            kd[i] = sqrt((Cxx[i] + Cyy[i])/Czz[i]);
            a1[i] = Qzx[i]/(kd[i]*Czz[i]); // imaginary
            b1[i] = Qzy[i]/(kd[i]*Czz[i]); //imaginary
            a2[i] = (Cxx[i] - Cyy[i])/(kd[i]*kd[i]*Czz[i]); // real
            b2[i] = 2.0*Cxy[i]/(kd[i]*kd[i]*Czz[i]); // real
            dir[i] = atan2(b1[i],a1[i]);
            dir_mean += dir[i];
       }
       print2out(dir,w+1,"out/dir.out",ido_dirmem);

       return_data->peak_direction = dir[peak_id];
       return_data->direction = dir_mean*2/(w+1);
//       return_data->spread = spread[peak_id];
       return_data->ratio =sqrt(Cxx[peak_id]/(Cyy[peak_id]+Czz[peak_id])); 
              
       debug("p",1.0/(peak_id*8.0/ww));
       // Significant wave height
       Hs = 4.0*sqrt(c11_sum*8.0/ww);
       debug("   Hs(direction) ",Hs);
       debug("   Dp ",return_data->peak_direction*180/pi);

       return(0);
}

// ********************************************************************************
// ********************************************************************************

int wave_direction_MEM(struct DATARAW *data_raw, struct RETDATA *return_data){
        
       int i,j,peak_id;
       long double Hs;
       long double x,y,z,x_o,y_o;
       long double temp, sum, alpha, e1_real, e1_imag, e2_real, e2_imag, y_real, y_imag;
       long double div[m+1];
       long double d[360], S[m+1][360], Sn[m+1][360], E[m+1][360];
       long double fft_roll[n_raw], fft_pitch[n_raw], fft_heave[n_raw];
       long double roll_real[n_raw/2+1], pitch_real[n_raw/2+1], heave_real[n_raw/2+1];
       long double roll_complex[n_raw/2+1], pitch_complex[n_raw/2+1], heave_complex[n_raw/2+1];
       long double Cxx[n_raw/2+1], Cyy[n_raw/2+1], Czz[n_raw/2+1];
       long double Cxy[n_raw/2+1], Czx[n_raw/2+1], Czy[n_raw/2+1];
       long double RP_complex[n_raw/2+1], Qzx[n_raw/2+1], Qzy[n_raw/2+1];
       long double kd[n_raw/2+1], a1[n_raw/2+1], a2[n_raw/2+1], b1[n_raw/2+1], b2[n_raw/2+1];
       long double c1_real[n_raw/2+1], c1_imag[n_raw/2+1], c2_real[n_raw/2+1], c2_imag[n_raw/2+1]; 
       long double p1_real[n_raw/2+1], p1_imag[n_raw/2+1], p2_real[n_raw/2+1], p2_imag[n_raw/2+1];
       long double x1_real[n_raw/2+1], x1_imag[n_raw/2+1], y1[n_raw/2+1];
       long double dir[n_raw/2+1];
       long double win[n_raw];
       long double peak;
       int gamma;
       long double alph, beta, t1;

       // define 10% cosine taper
       gamma = 5; t1 = n_raw/2-n_raw/(2*gamma); alph = 2.0*3.14159265*gamma/n_raw;
       if (gamma % 2 == 0) beta = 3.14159265; else beta = 0;
       for (i=0;i<n_raw/2;i++) {
            if (i<t1) win[i] = 1;
            else win[i] = 0.5*(1 + cos(alph*i + beta)); 
       }
       for (i=n_raw/2;i<n_raw;i++) win[i] = win[i-n_raw/2]; 
       for (i=0;i<n_raw/2;i++) win[i] = win[n_raw-i];

       for (i=0;i<n_raw;i++) {
           fft_roll[i] = data_raw->roll[i];
           fft_pitch[i] = data_raw->pitch[i];
           fft_heave[i] = data_raw->accel_vert[i];
//           demean(fft_roll,n_raw);
//           demean(fft_pitch,n_raw);
//           demean(fft_heave,n_raw);
//           trend_removal(fft_roll,n_raw);
//           trend_removal(fft_pitch,n_raw);
//           trend_removal(fft_heave,n_raw);
//           demean(fft_roll,n_raw);
//           demean(fft_pitch,n_raw);
//           demean(fft_heave,n_raw);
           fft_roll[i] *= win[i];
           fft_pitch[i] *= win[i];
           fft_heave[i] *= win[i];
       }
       print2out(fft_roll,n_raw,"out/fft_roll.out",ido_dirmem);
       print2out(fft_roll,n_raw,"out/fft_pitch.out",ido_dirmem);
       print2out(fft_roll,n_raw,"out/fft_heave.out",ido_dirmem);

       realft(fft_roll,n_raw,1);
       realft(fft_pitch,n_raw,1);
       realft(fft_heave,n_raw,1);

       // auto / cross correlations
       for (i=0,j=0; i<n_raw; i+=2,j++){
            roll_real[j] = fft_roll[i]; 
            pitch_real[j] = fft_pitch[i];
            heave_real[j] = fft_heave[i];
       }
       roll_real[n_raw/2] = 0;
       pitch_real[n_raw/2] = 0;
       heave_real[n_raw/2] = 0;

       roll_complex[0] = 0;
       roll_complex[1] = 0;
       pitch_complex[0] = 0;
       pitch_complex[1] = 0;
       heave_complex[0] = 0;
       heave_complex[1] = 0;
       for (i=3,j=2; i<n_raw; i+=2,j++){
            roll_complex[j] = fft_roll[i]; 
            pitch_complex[j] = fft_pitch[i];
            heave_complex[j] = fft_heave[i];
       }
       roll_complex[n_raw/2] = 0;
       pitch_complex[n_raw/2] = 0;
       heave_complex[n_raw/2] = 0;

       // Kuik et al.
       peak = 0;
       for (i=0;i<n_raw/2+1;i++){
            // Roll * conjugate(roll)
            Cxx[i] = roll_real[i]*roll_real[i] + roll_complex[i]*roll_complex[i];
            // Pitch * conjugate(pitch)
            Cyy[i] = pitch_real[i]*pitch_real[i] + pitch_complex[i]*pitch_complex[i];
            // Heave * conjugate(heave/)
            Czz[i] = heave_real[i]*heave_real[i] + heave_complex[i]*heave_complex[i];
            // roll * conjugate(pitch)
            Cxy[i] = roll_real[i]*pitch_real[i] + roll_complex[i]*pitch_complex[i];
            // roll * conjugate(heave)
            Czx[i] = roll_real[i]*heave_real[i] + roll_complex[i]*heave_complex[i]; // real
            Qzx[i] = heave_real[i]*roll_complex[i] - roll_real[i]*heave_complex[i]; // imaginary
            // pitch * conjugate(heave)
            Czy[i] = pitch_real[i]*heave_real[i] + pitch_complex[i]*heave_complex[i]; // real
            Qzy[i] = heave_real[i]*pitch_complex[i] - pitch_real[i]*heave_complex[i]; // imaginary
            if (peak<=Czz[i]) {
                peak = Czz[i];
                peak_id = i;
            }
       }

       // Kd
       for (i=0;i<n_raw/2+1;i++){
            // Kuik method
            kd[i] = sqrt((Cxx[i] + Cyy[i])/Czz[i]);
       
            // Moments (Kuik et al.)
            a1[i] = Qzx[i]/(kd[i]*Czz[i]); // imaginary
            b1[i] = Qzy[i]/(kd[i]*Czz[i]); //imaginary
            a2[i] = (Cxx[i] - Cyy[i])/(kd[i]*kd[i]*Czz[i]); // real
            b2[i] = 2.0*Cxy[i]/(kd[i]*kd[i]*Czz[i]); // real

            dir[i] = atan2(b1[i],a1[i])*180.0/pi;

            // Krogstad notation
            // d1 = a1, d2 = b1, d3 = a2, d4 = b2
//            c1_real[i] = -b1[i];
//            c1_imag[i] = a1[i];
//            c2_real[i] = a2[i];
//            c2_imag[i] = b2[i];
//            div[i] = (1 - pow(c1_real[i],2) - pow(c1_imag[i],2));
//            p1_real[i] = (c1_real[i] - c2_real[i]*c1_real[i] - c2_imag[i]*c1_imag[i])/div[i];
//            p1_imag[i] = (c1_imag[i] + c1_imag[i]*c2_real[i] - c1_real[i]*c2_imag[i])/div[i];
//            p2_real[i] = c2_real[i] - c1_real[i]*p1_real[i] + c1_imag[i]*p1_imag[i];
//            p2_imag[i] = c2_imag[i] - c1_real[i]*p1_imag[i] - c1_imag[i]*p1_real[i];
//            x1_real[i] = 1-p1_real[i]*c1_real[i]- p1_imag[i]*c1_imag[i] - p2_real[i]*c2_real[i] - p2_imag[i]*c2_imag[i];
//            x1_imag[i] = p1_real[i]*c1_imag[i] - p1_imag[i]*c1_real[i] + p2_real[i]*c2_imag[i] - p2_imag[i]*c2_real[i];
//
//            sum = 0;
//            for (j=0;j<360;j++) {
//                 d[j] += j;
//                 alpha = d[j]*pi/180;
//                 e1_real = cos(alpha);
//                 e1_imag = -sin(alpha);
//                 e2_real = cos(2*alpha);
//                 e2_imag = -sin(2*alpha);
//                 y_real = 1 - p1_real[j]*e1_real + p1_imag[j]*e1_imag - p2_real[j]*e2_real + p2_imag[j]*e2_imag;
//                 y_imag = -p1_imag[j]*e1_real - p1_real[j]*e1_imag -p2_real[j]*e2_imag - p2_imag[j]*e2_real;
//                 y1[j] = pow(y_real,2) + pow(y_imag,2);
//                 S[i][j] = x1_real[i]/y1[j];
//                 sum += S[i][j];
//            }
//
//            for (j=0;j<360;j++) {
//                 Sn[i][j] = S[i][j]/(2*sum);
//                 E[i][j] = Sn[i][j]*return_data->psd[i];
//            }
       
       }

       debug("p",1.0/(peak_id*4.0/(n_raw)));
       debug("dir_peak",dir[peak_id]);      
       print2out_2D(E,513,360,"out/E.out",ido_E);
       print2out(dir,n_raw/2+1,"out/dir_MEM.out",ido_dirmem);

       // Significant wave height
       for (i=0;i<n_raw/2+1;i++) Hs += Czz[i];
       Hs = 4.0*sqrt(Hs*4/(n_raw));
       debug("   Hs(direction) ",Hs);

       return(0);
}

// ********************************************************************************
// ********************************************************************************

int wave_direction(struct DATARAW *data_raw, struct RETDATA *return_data){

       int i,j;
       long double df = 1/(n_raw*dt_raw),omega;
       long double c12 = 0, q12 = 0;
       long double c13 = 0, q13 = 0;
       long double c11 = 0, c22 = 0, c33 = 0;
       long double fft_accel[n_raw], fft_ns[n_raw], fft_ew[n_raw];
       long double a1[n_raw/2], b1[n_raw/2];
       long double a2[n_raw/2], b2[n_raw/2];
       long double a3[n_raw/2], b3[n_raw/2];
       long double alpha12, alpha13;
       long double aa1, bb1;

       for (i=0; i<n_raw; i++) {
            fft_accel[i] = data_raw->accel_vert[i];
            fft_ns[i] = data_raw->pitch[i];
            fft_ew[i] = data_raw->roll[i];
       }

       realft(fft_accel,n_raw,1);
       realft(fft_ns,n_raw,1);
       realft(fft_ew,n_raw,1);

       for (i=0,j=0; i<n_raw; i+=2,j++){
            a1[j] = fft_accel[i]; 
            a2[j] = fft_ns[i];
            a3[j] = fft_ew[i];
       }

       b1[0] = 0;
       b1[1] = 0;
       b1[0] = 0;
       b1[1] = 0;
       b2[0] = 0;
       b2[1] = 0;
       b3[0] = 0;
       b3[1] = 0;
       for (i=3,j=2; i<n_raw; i+=2,j++){
            b1[j] = fft_accel[i]; 
            b2[j] = fft_ns[i];
            b3[j] = fft_ew[i];
       }

       for (i = 0; i<n_raw/2; i++){
            omega = 2.0*pi*i*4.0/(n_raw);
            c12 += a1[i]*a2[i] + b1[i]*b2[i]*-omega*omega;
            q12 += a1[i]*b2[i] - b1[i]*a2[i]*-omega*omega;
            c13 += a1[i]*a3[i] + b1[i]*b3[i]*-omega*omega;
            q13 += a1[i]*b3[i] - b1[i]*a3[i]*-omega*omega;
            c11 += a1[i]*a1[i] + b1[i]*b1[i]*omega*omega*omega*omega;
            c22 += a2[i]*a2[i] + b2[i]*b2[i];
            c33 += a3[i]*a3[i] + b3[i]*b3[i];
       }
//       c12 = c12*n_raw/4;
//       q12 = q12*n_raw/4;
//       c11 = c11*n_raw/4;
//       c22 = c22*n_raw/4;
//       c33 = c33*n_raw/4;

       alpha12 = atan2(c12,q12);
       alpha13 = atan2(c13,q13);

       aa1 = (c12*sin(alpha12) + q12*cos(alpha12))/sqrt(c11*(c22+c33));
       bb1 = (c13*sin(alpha13) + q13*cos(alpha13))/sqrt(c11*(c22+c33));

       return_data->direction = atan2(bb1,aa1);
       return_data->spread = sqrt(2-2*sqrt(pow(aa1,2) + pow(bb1,2)));
       c11 = a1[52]*a1[52] + b1[52]*b1[52];
       c22 = a2[52]*a2[52] + b2[52]*b2[52];
       c33 = a3[52]*a3[52] + b3[52]*b3[52];
       return_data->ratio = sqrt(c11/(c22+c33));
              
       return(0);
}

// ********************************************************************************
// ********************************************************************************

