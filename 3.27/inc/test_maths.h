#ifndef test_maths_h
#define test_maths_h

#define n_raw64 40960
#define n_rawh 5120

int trend_removal(long double *data, int n);
void despike(long double *data, int *spike_total, int *spike_max, int n, int nstd);
long double demean(long double *data, int n);  
int intgrt_press(long double accel[], long double disp[], float sr, int n_a, int n_s, long double f1, long double f2);
void realft(long double *data, unsigned long n, int isign);
void periodic_func(long double *data, long double period, long double amp, float dt, int n);
int peak_freq(long double *data, long double *max_f, long double *max_s, float dt, int n);
void decimate(long double *in, long double *out, float sr1, float sr2, int n1, int n2);
int test_spectrum_cycle(long double period, long double amp, float dt, int m); 
void spctrm_psd(long double *disp, long double *psd, int mm, int kk);
void spctrm(long double *in, long double *out, int mm, int kk, int n, float sample_rate);
void deterministic_analysis(long double disp[],long double hz_max[],long double hcm_max[],long double htm_max[],long double tz_mean[],long double hz_mean[]);

int cycle_decimate(long double period, long double amp, float dt, float dt_raw, int n);
int cycle_decimate_tuck(long double period, long double amp, float dt, float dt_raw, int n);
int test_decimate(int n, float dt, float dt_raw);
int test_integration(int n, int n_spec, float dt);
int test_deterministic_analysis(int n, int n_spec, float dt, int k, int m);
int test_sample_length();
int test_dt();
int test_spectrum();
int test_rpy();
int test_rpy_5min();
int test_detrend();
int test_despike();
int test_displacement();
int test_rig();
int test_tank();

#define verbose 0
#define verbose_error 1

#endif
