
// Define functions
int intgrt_press(long double accel[], long double disp[], float sr, int n_a, int n_s, long double f1, long double f2);
void rspns(long double R[], long double f[], int n, long double f1, long double f2);
void deterministic_analysis(long double disp[],long double hz_max[],long double hcm_max[],long double htm_max[],long double tz_mean[], long double hz_mean[]);
void spctrm_psd(long double *disp, long double *psd, int mm, int kk);
void spctrm_imu(long double *disp, long double *psd, int mm, int kk);
void spctrm(long double *in, long double *out, int mm, int kk, int n, float sample_rate);
int wave_direction(struct DATARAW *data_raw, struct RETDATA *return_data);
int wave_direction_FEM(struct DATARAW *data_raw, struct RETDATA *return_data);
int wave_direction_FEM_2hz(long double *heave, long double *pitch, long double *roll, struct RETDATA *return_data);
int wave_direction_FEM_2hz_accel(long double *accel, long double *pitch, long double *roll, struct RETDATA *return_data);
int wave_direction_MEM(struct DATARAW *data_raw, struct RETDATA *return_data);
int wave_direction_MEM_raw(struct DATARAW *data_raw, struct RETDATA *return_data);
int quality_control_fft_trend(long double *data,long double *std);

#ifdef COMMAND_LINE
// Input files
FILE *idi_d; // displacement 
FILE *idi_v; // angular velocity
// Output files
FILE *ido_o; // final ouput 
FILE *ido_rf; // response function (frequency domain)
FILE *ido_rt; // response function (time domain)
FILE *ido_df; // displacement full 
FILE *ido_v; // angular velocity 
#endif




