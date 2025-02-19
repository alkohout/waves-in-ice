
// Define functions
void mean_rpy(struct DATARAW *data_raw, struct RETDATA *return_data); 
void accel_orient_exact_imu(struct DATARAW *data_raw, struct RETDATA *return_data);
void accel_orient_exact_kist(struct DATARAW *data_raw, struct RETDATA *return_data);
int rpy(double *gyro, double *accel, double *magno);
int test_stats(long double *data, int n);
void calibrate(struct DATARAW *data_raw);
void calibrate_kist(long double *data, int n);
void calibrate_accel(long double *data, int n);
void gpsraw_2_gps8hz(struct DATARAW *data_raw, struct RETDATA *return_data);
int patch(long double *data, int n); 
int trend_removal(long double *data, int n);
int flat(long double *data, int *total, int *max, double qflg_max, int n );
void despike(long double *data, int *spike_total, int *spike_max, int n, int nstd);
int quality_control_fft(long double *data);
void quality_control_accel(struct DATARAW *data_raw, struct RETDATA *return_data);
void quality_control_gyro(struct DATARAW *data_raw, struct RETDATA *return_data);
void quality_control_magno(struct DATARAW *data_raw, struct RETDATA *return_data);
void quality_control_rollpitch(struct DATARAW *data_raw, struct RETDATA *return_data);
void quality_control_heading(struct DATARAW *data_raw, struct RETDATA *return_data);
void realft(long double *data, unsigned long n, int isign);
void calibrate_pos(long double *data, int n);
void calibrate_magno(long double *data, int n);
void calibrate_gyro(long double *data, int n);
int north_east(struct DATARAW *data_raw, struct RETDATA *return_data);

long double std_kist;
long double roll_t, pitch_t, yaw_t;
