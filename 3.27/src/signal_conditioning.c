/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

// XXX
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "accel_calib.h"
#include "util.h"
#include "main.h"
#include "signal_conditioning.h"
#include "process_data.h"

void signal_conditioning( struct DATARAW *data_raw, struct DATA2HZ *data_2hz,
                         struct RETDATA *return_data) {

    logger("Start signal_conditioning",0.0);

    int i,j; // index
    long double temp, mean_yaw = 0.0, mean_yaw_sq = 0.0;

    // Initialize 
    return_data->qflg_kist = 0;
    return_data->qflg_accel = 0;
    return_data->qflg_rollpitch = 0;
    return_data->qflg_imu = 0;
    return_data->qflg_head = 0;
    return_data->qflg_open_water = 0;

    calibrate(data_raw);

    print2out(data_raw->accel_raw_kist,n_raw,"out/accel_raw_k.out",ido_arx);
    print2out(data_raw->accel_raw_x,n_raw,"out/accel_raw_x.out",ido_arx);
    print2out(data_raw->accel_raw_y,n_raw,"out/accel_raw_y.out",ido_ary);
    print2out(data_raw->accel_raw_z,n_raw,"out/accel_raw_z.out",ido_arz);
    print2out(data_raw->gyro_raw_x,n_raw,"out/gyro_raw_x.out",ido_arx);
    print2out(data_raw->gyro_raw_y,n_raw,"out/gyro_raw_y.out",ido_ary);
    print2out(data_raw->gyro_raw_z,n_raw,"out/gyro_raw_z.out",ido_arz);
    print2out(data_raw->magno_raw_x,n_raw,"out/magno_raw_x.out",ido_arx);
    print2out(data_raw->magno_raw_y,n_raw,"out/magno_raw_y.out",ido_ary);
    print2out(data_raw->magno_raw_z,n_raw,"out/magno_raw_z.out",ido_arz);
    print2out(data_raw->roll,n_raw,"out/roll_raw.out",ido_roll);
    print2out(data_raw->pitch,n_raw,"out/pitch_raw.out",ido_pitch);
    print2out(data_raw->yaw,n_raw,"out/yaw_raw.out",ido_yaw);

    quality_control(data_raw, return_data);

    //yaw standard deviation
    for (i=0;i<n_raw;i++) {
         mean_yaw += data_raw->yaw[i]+(2.0*pi);
         mean_yaw_sq += pow(data_raw->yaw[i]+(2.0*pi),2);
    }
    mean_yaw /= n_raw;
    mean_yaw_sq /= n_raw;
    return_data->yaw_mean = mean_yaw;
    return_data->std_yaw = sqrt(mean_yaw_sq-pow(mean_yaw,2));
    if (return_data->std_yaw*180.0/pi > std_yaw_limit) return_data->qflg_open_water=1;

    // Vertical acceleration
    if (return_data->qflg_kist == 0) {
       if (return_data->qflg_imu == 0) {
           logger("Kistler Passed. IMU Passed. Vertical accelertion corrected using IMU's roll and pitch",0.0);
           accel_orient_exact_kist(data_raw,return_data); 
       } else {
           logger("Kistler Passed. IMU Failed. Kistler assumed vertical",0.0);
           for (i=0;i<n_raw;i++) data_raw->accel_vert[i] =  data_raw->accel_raw_kist[i];
       }
    } else {
       logger("Kistler Failed. IMU used to calculate vertical acceleration",0.0);
       accel_orient_exact_imu(data_raw,return_data); 
    }

    // Correcting roll and pitch relative to true north 
    if (return_data->qflg_head == 0) gpsraw_2_gps8hz(data_raw,return_data);
    north_east(data_raw, return_data);
    quality_control_fft(data_raw->roll);
    quality_control_fft(data_raw->pitch);
    return_data->qflg_mean_removed_imu = demean(data_raw->accel_vert,n_raw);
    quality_control_fft(data_raw->accel_vert);

    // Downsample from raw hz to 2 hz
    decimate(data_raw->accel_vert,data_2hz->accel_z,dt_raw,dt,n_raw,n_accel);
    demean(data_2hz->accel_z,n_accel);
    decimate(data_raw->roll,data_2hz->roll,dt_raw,dt,n_raw,n_accel);
    demean(data_2hz->roll,n_accel);
    decimate(data_raw->pitch,data_2hz->pitch,dt_raw,dt,n_raw,n_accel);
    demean(data_2hz->pitch,n_accel);

    // Print to output
    print2out(data_raw->accel_vert,n_raw,"out/accel_vert.out",ido_av);
    print2out(data_raw->accel_raw_kist,n_raw,"out/accel_clean_k.out",ido_ark);
    print2out(data_raw->accel_raw_x,n_raw,"out/accel_clean_x.out",ido_arx);
    print2out(data_raw->accel_raw_y,n_raw,"out/accel_clean_y.out",ido_ary);
    print2out(data_raw->accel_raw_z,n_raw,"out/accel_clean_z.out",ido_arz);
    print2out(data_raw->gyro_raw_x,n_raw,"out/gyro_clean_x.out",ido_arx);
    print2out(data_raw->gyro_raw_y,n_raw,"out/gyro_clean_y.out",ido_ary);
    print2out(data_raw->gyro_raw_z,n_raw,"out/gyro_clean_z.out",ido_arz);
    print2out(data_raw->magno_raw_x,n_raw,"out/magno_clean_x.out",ido_arx);
    print2out(data_raw->magno_raw_y,n_raw,"out/magno_clean_y.out",ido_ary);
    print2out(data_raw->magno_raw_z,n_raw,"out/magno_clean_z.out",ido_arz);
    print2out(data_2hz->roll,n_accel,"out/roll_2hz.out",ido_r2);
    print2out(data_2hz->pitch,n_accel,"out/pitch_2hz.out",ido_p2);
    print2out(data_2hz->accel_z,n_accel,"out/accel_2hz.out",ido_a2);

    #ifdef SEND_RAW
      ido_raw = fopen("out/raw.out","w");
      if (ido_raw == NULL) perror("Error raw.out");
      for (i=0;i<n_accel;i++) fprintf(ido_raw,"%LF %LF %LF \n",data_2hz->accel_z[i],data_2hz->roll[i],data_2hz->pitch[i]); 
      fclose(ido_raw);
    #endif 
  
    logger("End signal_conditioning",0.0);

    return;
}
