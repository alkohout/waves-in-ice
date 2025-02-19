
#ifndef main_h
#define main_h

#include <stdio.h>
#include <time.h>
#include <math.h>

#define m 256 // minimum - 32
#define k 2 // minimum - 1
#define n_spec (2*k+1)*m
#define arduino 1 //
// Makefile #define n_accel 1280 // minimum - 512 // acceleration size post digitisation
// Makefile #define dt 0.5 // 2 Hz with m = 256, test_maths shows valid dt are .25 or .5 only
// Makefile #define dt_raw 0.125 // 8 Hz
// Makefile #define dt_raw64 0.015625 //
// Makefile #define n_raw 5120 //(int)(n_accel/(dt_raw/dt))  // size of raw data
// Makefile #define n_raw64 40960 //(int)(n_accel/(dt_raw64/dt))  // size of raw 64 Hz data
// Makefile #define n_accelfft 2048 // test_maths shows minimum 1024
// Makefile #define n_psd 55 // size psd after frequency averaging
// Makefile #define n_mom 7 // no. of moments calculated
// Makefile #define n_calib 4  // no. of values read from calibrate.in
// Makefile #define g 9.80665 // gravity of earth m/s^2
// Makefile #define pi 3.141592653589793
// Makefile #define rmv_ramp 1 // 1 = first 2000 pts of raw data zero
// Makefile #define std_kist_min .1
// Makefile #define rspns_f1 0.01 // response cut off minimum frequency first look
// Makefile #define rspns_f2 0.02 // response cut off maximum frequency first look
// Makefile #define rspns_f3 0.03 // response cut off maximum frequency default backup
// Makefile #define rspns_f4 0.04 // response cut off maximum frequency default backup
// Makefile #define rspns_f5 0.03 // high frequency noise segment maximum frequency - pd.c
// Makefile #define rspns_f6 0.5  // low frequency noise segment minimum frequency  - pd.c
// Makefile #define rspns_f7 0.04 // Lower bound for directional analysis (Hz) - direction.c
// Makefile #define rspns_f8 0.25 // Upper bound for directional analysis (Hz) - direction.c
// Makefile #define declination 17.08674 // Arctic
// Makefile #define std_yaw_limit 1.0 // Min yaw standard deviation for open water flag
// Makefile #define nstd_kist 3 // Standard deviation multiplier in despike
// Makefile #define nstd_imu 6 // Standard deviation multiplier in despike
// Makefile #define nstd_magno 6 // Standard deviation multiplier in despike
// Makefile #define nstd_gyro 6 // Standard deviation multiplier in despike
// Makefile #define nstd_pos 6 // Standard deviation multiplier in despike
// Makefile #define noise_factor 0.2 // Noise factor in identify_low_frequency_noise
// Makefile #define declination 0.0 // if not sure - can do it post data collection

long double calib[n_calib];

struct DATARAW {
        unsigned long long start;
        unsigned long long gps_time[n_raw];
	long double accel_raw64_kist[n_raw64];
	long double accel_raw64_x[n_raw64];
	long double accel_raw64_y[n_raw64];
	long double accel_raw64_z[n_raw64];
	long double gyro_raw64_x[n_raw64];
	long double gyro_raw64_y[n_raw64];
	long double gyro_raw64_z[n_raw64];
	long double magno_raw64_x[n_raw64];
	long double magno_raw64_y[n_raw64];
	long double magno_raw64_z[n_raw64];
	long double accel_raw_kist[n_raw];
        long double accel_vert[n_raw];
	long double accel_raw_x[n_raw];
	long double accel_raw_y[n_raw];
	long double accel_raw_z[n_raw];
	long double gyro_raw_x[n_raw];
	long double gyro_raw_y[n_raw];
	long double gyro_raw_z[n_raw];
	long double magno_raw_x[n_raw];
	long double magno_raw_y[n_raw];
	long double magno_raw_z[n_raw];
	long double inclo_x[n_raw];
	long double inclo_y[n_raw];
        long double roll64[n_raw64];
        long double pitch64[n_raw64];
        long double yaw64[n_raw64];
        long double roll[n_raw];
        long double pitch[n_raw];
        long double yaw[n_raw];
        long double rolla[n_raw];
        long double pitcha[n_raw];
        long double yawa[n_raw];
        long double roll_arduino;
        long double pitch_arduino;
        long double yaw_arduino;
//        long double yaw_mean;
        long double gps_heading;
        long double gps_headings[n_raw];
        long double gps_headings_8hz[n_raw];
};

// Data at 2HZ (converted m**2, 2HZ etc)
struct DATA2HZ {
	long double accel_kist[n_accel];
	long double accel_z[n_accel];
        long double accel_zeros[n_accelfft];
        long double roll[n_accel];
        long double pitch[n_accel];
        long double yaw[n_accel];
        long double heave[n_accel];
        long double gps_headings[n_accel];
};

// Return data
struct RETDATA {
        long double hz_max[2*k];
        long double hcm_max[2*k];
        long double htm_max[2*k];
        long double tz_mean[2*k];
        long double hz_mean[2*k];
        long double hmod;
        long double tp;
	long double psd[n_psd];
	long double moment[n_mom];
        long double roll_mean;
        long double pitch_mean;
        long double yaw_mean;
        long double ns[n_raw];
        long double ew[n_raw];
        long double peak_direction;
        long double direction;
        long double spread;
        long double ratio;
        long double hs_dir;
        long double std_yaw;
        long double qflg_mean_removed;
        long double qflg_mean_removed_imu;
        long double qflg_tot_pow_imu;
        long double a;
        long double b;
        long double r;
        long double f2;
        long double nstd;
        int direction_full[m+1];
        int qflg_kist;
        int qflg_imu;
        int qflg_pkist;
        int qflg_accel;
	int qflg_gyro;
	int qflg_magno;
        int qflg_rollpitch;
        int qflg_head;
        int qflg_open_water;
        int n_gps;
};

// Define global functions
void quality_control(struct DATARAW *data_raw, struct RETDATA *return_data);
int quality_control_kist(struct DATARAW *data_raw, struct RETDATA *return_data);
void decimate(long double *in, long double *out, float sr1, float sr2, int n1, int n2);
long double demean(long double *data, int n);

// Define functions
//int ProcessData(struct DATARAW *data_raw, struct DATA2HZ *data, struct RETDATA *ret);
//void signal_conditioning(struct DATARAW *data_raw, struct DATA2HZ *data_2z,
//                        struct RETDATA *ret);

// Input  files
FILE *idi_raw; // imu data
FILE *idi_calib; //
FILE *idi_meta;
FILE *idi_gps;

// Output files
FILE *ido_o;
FILE *ido_oqc;
FILE *ido_ark;
FILE *ido_arx;
FILE *ido_arx64;
FILE *ido_ary;
FILE *ido_arz;
FILE *ido_grx;
FILE *ido_gry;
FILE *ido_grz;
FILE *ido_mrx;
FILE *ido_mry;
FILE *ido_mrz;
FILE *ido_irx;
FILE *ido_iry;
FILE *ido_roll;
FILE *ido_pitch;
FILE *ido_yaw;
FILE *ido_rolla;
FILE *ido_pitcha;
FILE *ido_yawa;
FILE *ido_rollm;
FILE *ido_pitchm;
FILE *ido_yawm;
FILE *ido_av;
FILE *ido_rs;
FILE *ido_ps;
FILE *ido_hs;
FILE *ido_gps;

FILE *ido_a2k;
FILE *ido_a2z;
FILE *ido_r2;
FILE *ido_p2;
FILE *ido_a2;

FILE *ido_bin;
FILE *ido_f2;
FILE *ido_f8;
FILE *ido_fark;
FILE *ido_fa2k;
FILE *ido_farz;
FILE *ido_fa2z;
FILE *ido_tdk;
FILE *ido_tdz;
FILE *ido_sk;
FILE *ido_sz;
FILE *ido_momz;
FILE *ido_momk;
FILE *ido_E;
FILE *ido_dt;
FILE *ido_dir;
FILE *ido_od;
FILE *ido_dirmem;
FILE *ido_favg;
FILE *ido_raw;

#endif
