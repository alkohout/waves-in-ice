/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

// Program process_data 
/* XXXvim: set tabstop=2 shiftwidth=2 expandtab: */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include "accel_calib.h"
#include "util.h"
#include "main.h"

// Define functions
int ProcessData(struct DATARAW *data_raw, struct DATA2HZ *data, struct RETDATA *ret);
void signal_conditioning(struct DATARAW *data_raw, struct DATA2HZ *data_2z,
                        struct RETDATA *ret);

// ********************************************************************************
// ********************************************************************************

struct DATARAW data_raw;
struct DATA2HZ data_2hz;
struct RETDATA ret;

int main(void)
{

  clock_t start;
  int i,j,id[n_raw],cnt;			// loop counter
  long double retdata[80];	// XXX better size
  long double temp;
  char gps_def;
  char buff[500];

  
  mkdir("out",S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  mkdir("log", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  start = clock();
  #ifdef COMMAND_LINE 
  debug("-- START processing data", 0);
  #endif

  // Debugging
  ido_l = fopen("log/process_data.log","w");
  if (ido_l == NULL) perror("Error process_data.log");
  logger("Start main",0.0);
  logger("",0.0);

  // Open calibration file
  logger("Read calibration file",0.0);
  idi_calib = fopen("in/calibrate.in","r");
  if (idi_calib == NULL) perror("Error calibrate.in");
  for ( i=0; i<n_calib; i++) fscanf(idi_calib,"%*s%LF",&calib[i]); 
  fclose(idi_calib);
  logger("Calibration file read",0.0);

  // Read meta file 
  logger("Read meta file",0.0);
  data_raw.gps_heading = -9999.99;
  /*
  idi_meta = fopen("in/meta.in","r");
  if (idi_meta == NULL) perror("Error reading meta.in");
  fgets(buff,100,idi_meta);
  sscanf(buff,"%*s %llu\n",&data_raw.start);
  fgets(buff,100,idi_meta);
  sscanf(buff,"%*s %LF",&data_raw.gps_heading);
  fclose(idi_meta);
  logger("Meta file read",0.0);

  // Read GPS compass
  cnt=0;
  for(i=0;i<n_raw;i++){
      data_raw.gps_time[i] = 9999;
      data_raw.gps_headings[i] = -9999.9999;
  }
  logger("Read GPS headings",0.0);
  idi_gps = fopen("in/compass.log","r");
  if (idi_gps != NULL) {
      for(i=0;i<n_raw;i++){
          data_raw.gps_time[i] = 9999;
          data_raw.gps_headings[i] = -9999.9999;
          fgets(buff,500,idi_gps); 
          sscanf(buff,"%*llu%*c%c\n",&gps_def); 
          if (gps_def == 'D') {
              sscanf(buff,"%llu %*s %*8c%*d%*11c%*d%*8c%*LF%*9c%*LF%*9c%*LF%*19c%*LF%*12c%*d%*12c%*LF%*11c%LF\n",
                          &data_raw.gps_time[cnt],&data_raw.gps_headings[cnt]); 
              cnt++;
          }
      }
  }
  fclose(idi_gps);
  ret.n_gps = cnt;
  logger("GPS headings read",0.0);
  */

  // Read raw data
  logger("Read raw data",0.0);
  #ifdef SAMPLE_64
    logger("Sampling at 64 Hz ",0.0);
    idi_raw = fopen("in/raw.log","r");
    if (idi_raw == NULL) perror("Error raw.log");
    for ( i=0; i<n_raw64; i++) {
          fscanf(idi_raw,"%*F%*c%*F%*c%*F%*c\
                            %LF%*c%*F%*c%*F%*c\
                            %LF%*c%LF%*c%LF%*c%LF%*c%LF%*c%LF%*c%LF%*c%LF%*c%LF%*c\
                            %LF%*c%LF%*c%LF%*c\
                          ",
          &data_raw.accel_raw64_kist[i],
          &data_raw.accel_raw64_x[i],
          &data_raw.accel_raw64_y[i],
          &data_raw.accel_raw64_z[i],
          &data_raw.gyro_raw64_x[i],
          &data_raw.gyro_raw64_y[i],
          &data_raw.gyro_raw64_z[i],
          &data_raw.magno_raw64_x[i],
          &data_raw.magno_raw64_y[i],
          &data_raw.magno_raw64_z[i],
          &data_raw.roll64[i],
          &data_raw.pitch64[i],
          &data_raw.yaw64[i]); 
    }

    decimate(data_raw.accel_raw64_kist,data_raw.accel_raw_kist,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.accel_raw64_x,data_raw.accel_raw_x,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.accel_raw64_y,data_raw.accel_raw_y,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.accel_raw64_z,data_raw.accel_raw_z,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.gyro_raw64_x,data_raw.gyro_raw_x,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.gyro_raw64_y,data_raw.gyro_raw_y,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.gyro_raw64_z,data_raw.gyro_raw_z,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.magno_raw64_x,data_raw.magno_raw_x,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.magno_raw64_y,data_raw.magno_raw_y,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.magno_raw64_z,data_raw.magno_raw_z,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.roll64,data_raw.roll,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.pitch64,data_raw.pitch,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;
    decimate(data_raw.yaw64,data_raw.yaw,1.0/64.0,1.0/8.0,n_raw64,n_raw) ;

    // to test decimate
    print2out(data_raw.accel_raw64_x,n_raw64,"out/accel_raw64_x.out",ido_arx64);
    print2out(data_raw.accel_raw_x,n_raw,"out/accel_raw8_x.out",ido_arx);

    fclose(idi_raw);
  #else
    logger("Sampling at 8 Hz ",0.0);
    idi_raw = fopen("in/raw.log","r");
    if (idi_raw == NULL) perror("Error raw.log");
    for ( i=0; i<n_raw; i++) {
		  				//     i     milli   error
		  				// a   1*    2*     3*		ignored
						//     Kiss IncX   IncY
				  		// b   1    2*     3*
						//     Ax   Ay    Az   Gx    Gy    Gz    Mx    My    Mz
						// c   1    2     3    4     5     6     7     8     9
						//     Px   Py    Pz
						// d   1    2     3
          fscanf(idi_raw,"%*F%*c%*F%*c%*F%*c\
                            %LF%*c%*F%*c%*F%*c\
                            %LF%*c%LF%*c%LF%*c%LF%*c%LF%*c%LF%*c%LF%*c%LF%*c%LF%*c\
                            %LF%*c%LF%*c%LF%*c\
                          ",
			// b1
          &data_raw.accel_raw_kist[i],

		  // c 1 2 3
          &data_raw.accel_raw_x[i],
          &data_raw.accel_raw_y[i],
          &data_raw.accel_raw_z[i],
		  // c 4 5 6
          &data_raw.gyro_raw_x[i],
          &data_raw.gyro_raw_y[i],
          &data_raw.gyro_raw_z[i],
		  // c 7 9 0
          &data_raw.magno_raw_x[i],
          &data_raw.magno_raw_y[i],
          &data_raw.magno_raw_z[i],
		  // d 1 2 3
          &data_raw.roll[i],
          &data_raw.pitch[i],
          &data_raw.yaw[i]); 
    }
    fclose(idi_raw);
  #endif
  logger("Raw data read",0.0);

  logger("",0.0);
  signal_conditioning(&data_raw, &data_2hz, &ret);
  logger("",0.0);
  ProcessData(&data_raw, &data_2hz, &ret);
  logger("",0.0);

  logger("End main",0.0);
  #ifdef COMMAND_LINE 
  debug("   CPU time", (clock()-start)/(long double)CLOCKS_PER_SEC);
  debug("-- END   processing data ", 0);
  #endif

  return 0;

} // end main

