/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nrutil.h"
#include "util.h"

#define NRANSI
#define SWAP(a,b) tempr=(a);(a)=(b);(b)=tempr

void convlv(long double data[], unsigned long n, long double respns[], unsigned long m, int isign, long double ans[]) ;
int spctrm_psd(long double disp[], long double p[], int m, int k) ;
void spctrm_imu(long double disp[], long double p[], int m, int k) ;
void spctrm(long double disp[], long double p[], int m, int k, int n, float sample_rate) ;
void realft(long double data[], unsigned long n, int isign);
void four1(long double data[], unsigned long nn, int isign);
int trend_removal(long double *data, int n);
long double demean(long double *data, int n);  
void fft_real(long double data_in[], long double data_out[], int n);

// ********************************************************************************
// ********************************************************************************

void convlv(long double data[], unsigned long n, long double respns[], unsigned long m,
        int isign, long double ans[]) {

        unsigned long i,no2;
        long double dum,mag2;
        long double *fft;

        for (i=0;i<n;i++) fft[i] = 0.0;
//        fft=dvector(1.0,n<<1);
        for (i=1;i<=(m-1)/2;i++)
                respns[n+1-i]=respns[m+1-i];
        for (i=(m+3)/2;i<=n-(m-1)/2;i++)
                respns[i]=0.0;

        for (i=0; i<n; i++) {
               fft[i] = data[i];
               ans[i] = respns[i];
        }
        realft(fft,n,1);
        realft(ans,n,1);

        no2=n>>1;
        for (i=2;i<=n+2;i+=2) {
                if (isign == 1) {
                        ans[i-1]=(fft[i-1]*(dum=ans[i-1])-fft[i]*ans[i])/no2;
                        ans[i]=(fft[i]*dum+fft[i-1]*ans[i])/no2;
                } else if (isign == -1) {
                        if ((mag2=SQR(ans[i-1])+SQR(ans[i])) == 0.0)
                                nrerror("Deconvolving at response zero in convlv");
                        ans[i-1]=(fft[i-1]*(dum=ans[i-1])+fft[i]*ans[i])/mag2/no2;
                        ans[i]=(fft[i]*dum-fft[i-1]*ans[i])/mag2/no2;
                } else nrerror("No meaning for isign in convlv");
        }
        ans[2]=ans[n+1];
        realft(ans,n,-1);
        //free_dvector(fft,1,n<<1);
}

// ********************************************************************************
/******************************************************************************
 * Function: spctrm_psd
 * Purpose: Calculates Power Spectral Density using Welch's method with
 *          overlapping segments and 10% cosine tapering
 * 
 * Parameters:
 *   disp[] - Input displacement time series data
 *   p[]    - Output array for power spectral density values
 *   m      - Half the segment length (full segment = 2m)
 *   k      - Number of segments to average (determines overlap)
 * 
 * Returns:
 *   0 on success
 * 
 * Notes:
 *   - Applies detrending and demeaning to each segment
 *   - Uses 10% cosine taper window
 *   - Includes overlap processing for improved spectral estimates
 *   - Normalizes output by window power and number of segments
 * 
 *****************************************************************************/
// ********************************************************************************
int spctrm_psd(long double disp[], long double p[], int m, int k) {

        FILE *id;
        int i=0,ii=0,j=0,count=0;
        int mm=2*m;
        int gamma;
        int cl = 1;
        float r=0.0;
        long double win[mm], sumw=0;
        long double dp[mm], psd[2*k][m+1];
        long double alpha, beta, t1;
        long double w1[mm];


        // Intialize 
        for (i=0;i<mm;i++) dp[i] = 0.0;
        for (i=0;i<m+1;i++) {
             for(j=0;j<2*k;j++) psd[j][i] = 0.0;
             p[i] = 0.0;
        }

                /*
                 * SCOTT IDEA

                 
                // create first half
                //                 win [ 0..511 ] = new values
        for (i=0;i<m;i++) {
             if (i<t1) win[i + m] = 1;
             else win[i + m] = 0.5*(1 + cos(alpha*i + beta)); 

             if (i<t1) win[m - i] = 1;
             else win[m - i] = 0.5*(1 + cos(alpha*i + beta)); 
        }

                */

        // define 10% cosine taper
        gamma = 5; t1 = m-m/gamma; alpha = 2.0*3.14159265*gamma/mm;
        if (gamma % 2 == 0) beta = 3.14159265; else beta = 0;
        for (i=0;i<m;i++) {
             if (i<t1) win[i] = 1;
             else win[i] = 0.5*(1 + cos(alpha*i + beta)); 
        }
        for (i=m;i<mm;i++) win[i] = win[i-m]; 
        for (i=0;i<m;i++) win[i] = win[mm-1-i];
        for (i=0;i<mm;i++) sumw += pow(win[i],2.0);
        for (i=0;i<mm;i++) if(isnan(win[i])) debug("nan",i);

        for (j=0,count=0; j<2*k; j++, count+=m) {
             for (i=0;i<mm;i++) {
                  dp[i] = disp[count+i];
                  demean(dp,i);
                  trend_removal(dp,mm);
                  demean(dp,i);
                  w1[i] = win[i]*dp[i];
             }
             realft(w1,mm,1);
             psd[j][0] = pow(w1[0],2); psd[j][m] = pow(w1[1],2);
             for (i=2,ii=1; i<mm-1; i+=2,ii++) {
                  if (ii>=m+1) {
                      fprintf(stderr,"ERROR: out of bounds in spctrm_psd. Exiting.\n");
                      exit(0);
                  }
                  if (i>=2*m-1) {
                      fprintf(stderr,"ERROR: out of bounds in spctrm_psd. Exiting.\n");
                      exit(0);
                  }
                  psd[j][ii] = 2*(pow(w1[i],2) + pow(w1[i+1],2)); 
             }
        }

        for (i=0; i<m+1; i++) {
             p[i]=0.0;
             for (j=0; j<2*k; j++) p[i]+=psd[j][i];
             p[i] /= (2.0*k*mm*sumw);
//             if (isnan(p[i])) logger("ERROR: NAN's in spctrm_psd",1.0);
        }

        return(0);
}

// ********************************************************************************
// ********************************************************************************

void spctrm(long double in[], long double out[], int m2, int kk, int n, float sample_rate) {

        FILE *id;
        int i,ii,j,count;
        int mm=2*m2;
        int gamma;
        int cl = 1;
        long double win[mm], sumw=0;
        long double dp[mm], psd[2*kk][mm];
        long double alpha, beta, t1;
        long double w1[mm];

        for (i=0;i<mm;i++) {
             dp[i] = 0.0;
             out[i] = 0.0;
        }

        // define 10% cosine taper
        gamma = 5; t1 = m2-m2/gamma; alpha = 2.0*3.14159265*gamma/mm;
        if (gamma % 2 == 0) beta = 3.14159265; else beta = 0;
        for (i=0;i<m2;i++) {
             if (i<t1) win[i] = 1;
             else win[i] = 0.5*(1 + cos(alpha*i + beta)); 
        }
        for (i=m2;i<mm;i++) win[i] = win[i-m2]; 
        for (i=0;i<m2;i++) win[i] = win[mm-1-i];
        for (i=0;i<mm;i++) sumw += win[i];

        for (j=0,count=0; j<2*kk; j++, count+=m2) {
             for (i=0;i<mm;i++) {
                  dp[i] = in[count+i];
                  demean(dp,mm);
                  trend_removal(dp,mm);
                  demean(dp,mm);
                  w1[i] = win[i]*dp[i];
                  if (count+i>=n) {
                      fprintf(stderr,"ERROR: out of bounds in spctrm. Exiting.\n");
                      exit(0);
                  }
             }
             realft(w1,mm,1);
             for (i=0; i<mm; i++) {
                    psd[j][i] = w1[i]; 
             }
        }

        for (i=0; i<mm; i++) {
             out[i]=0;
             for (j=0; j<2*kk; j++) out[i]+=psd[j][i];
             out[i] /= (2*kk*mm)*sample_rate;
        }

}

// ********************************************************************************
// ********************************************************************************

void realft(long double data[], unsigned long n, int isign)
// 31/03/2012 change by AK
// Index fix by introducing data_temp
//
{
        unsigned long i,i1,i2,i3,i4,np3;
        long double c1=0.5,c2,h1r,h1i,h2r,h2i;
        long double wr,wi,wpr,wpi,wtemp,theta;
        long double data_temp[n+1];

        for (i=0;i<n;i++) data_temp[i+1] = data[i];

        theta=3.141592653589793/(long double) (n>>1);
        if (isign == 1) {
                c2 = -0.5;
                four1(data_temp,n>>1,1);
        } else {
                c2=0.5;
                theta = -theta;
        }
        wtemp=sin(0.5*theta);
        wpr = -2.0*wtemp*wtemp;
        wpi=sin(theta);
        wr=1.0+wpr;
        wi=wpi;
        np3=n+3;
        for (i=2;i<=(n>>2);i++) {
                i4=1+(i3=np3-(i2=1+(i1=i+i-1)));
                h1r=c1*(data_temp[i1]+data_temp[i3]);
                h1i=c1*(data_temp[i2]-data_temp[i4]);
                h2r = -c2*(data_temp[i2]+data_temp[i4]);
                h2i=c2*(data_temp[i1]-data_temp[i3]);
                data_temp[i1]=h1r+wr*h2r-wi*h2i;
                data_temp[i2]=h1i+wr*h2i+wi*h2r;
                data_temp[i3]=h1r-wr*h2r+wi*h2i;
                data_temp[i4] = -h1i+wr*h2i+wi*h2r;
                wr=(wtemp=wr)*wpr-wi*wpi+wr;
                wi=wi*wpr+wtemp*wpi+wi;
                if ( (i1>=n+1) || (i2>=n+1) || (i3>=n+1) || (i4>=n+1) ) { 
                      fprintf(stderr,"ERROR: out of bounds in realft. Exiting.\n");
                      exit(0);
                }
        }
        if (isign == 1) {
                data_temp[1] = (h1r=data_temp[1])+data_temp[2];
                data_temp[2] = h1r-data_temp[2];
        } else {
                data_temp[1]=c1*((h1r=data_temp[1])+data_temp[2]);
                data_temp[2]=c1*(h1r-data_temp[2]);
                four1(data_temp,n>>1,-1);
        }
        for(i=0;i<n;i++) data[i] = data_temp[i+1];
        return;
}

// ********************************************************************************
// ********************************************************************************

void four1(long double data[], unsigned long nn, int isign)
{
        unsigned long n,mmax,m,j,istep,i;
        long double wtemp,wr,wpr,wpi,wi,theta;
        long double tempr,tempi;

        n=nn << 1;
        j=1;
        for (i=1;i<n;i+=2) {
                if (j > i) {
                        SWAP(data[j],data[i]);
                        SWAP(data[j+1],data[i+1]);
                }
                m=nn;
                while (m >= 2 && j > m) {
                        j -= m;
                        m >>= 1;
                }
                j += m;
        }
        mmax=2;
        while (n > mmax) {
                istep=mmax << 1;
                theta=isign*(6.28318530717959/mmax);
                wtemp=sin(0.5*theta);
                wpr = -2.0*wtemp*wtemp;
                wpi=sin(theta);
                wr=1.0;
                wi=0.0;
                for (m=1;m<mmax;m+=2) {
                        for (i=m;i<=n;i+=istep) {
                                j=i+mmax;
                                tempr=wr*data[j]-wi*data[j+1];
                                tempi=wr*data[j+1]+wi*data[j];
                                data[j]=data[i]-tempr;
                                data[j+1]=data[i+1]-tempi;
                                data[i] += tempr;
                                data[i+1] += tempi;
                        }
                        wr=(wtemp=wr)*wpr-wi*wpi+wr;
                        wi=wi*wpr+wtemp*wpi+wi;
                }
                mmax=istep;
        }
}

// ********************************************************************************
// ********************************************************************************

void fft_real(long double data_in[], long double data_out[], int n)
{
    int i,j;

    realft(data_in,n,1);
    data_out[0] = 2.0/n*pow(data_in[0],2); 
    data_out[n/2] = 2.0/n*pow(data_in[1],2);
    for (i=2,j=1; i<n/2-1; i+=2,j++) data_out[j] = 2.0/n*(pow(data_in[i],2) + pow(data_in[i+1],2));

}
// ********************************************************************************
// ********************************************************************************

void spctrm_imu(long double disp[], long double p[], int m, int k) {

        FILE *id;
        int i,ii,j,count;
        int mm=2*m;
        int gamma;
        int cl = 1;
        long double win[2*m+1], sumw=0.0;
        long double dp[2*m], psd[2*k][m+1];
        long double alpha, beta, t1;
        long double w1[2*m];

        // define 10% cosine taper
        gamma = 5; t1 = m-m/gamma; alpha = 2.0*3.14159265*gamma/mm;
        if (gamma % 2 == 0) beta = 3.14159265; else beta = 0;
        for (i=0;i<m;i++) {
             if (i<t1) win[i] = 1;
             else win[i] = 0.5*(1 + cos(alpha*i + beta)); 
        }
        for (i=m;i<mm;i++) win[i] = win[i-m]; 
        for (i=0;i<m;i++) win[i] = win[mm-1-i];
        for (i=0;i<mm;i++) sumw += pow(win[i],2.0);

        for (j=0,count=0; j<2*k; j++, count+=m) {
             for (i=0;i<mm;i++) {
                  dp[i] = disp[count+i];
                  demean(dp,i);
                  trend_removal(dp,mm);
                  demean(dp,i);
                  w1[i] = win[i]*dp[i];
             }
             realft(w1,mm,1);
             psd[j][0] = pow(w1[0],2); psd[j][m] = pow(w1[1],2);
             for (i=2,ii=1; i<mm-1; i+=2,ii++) {
                  if (ii>=m+1) {
                      fprintf(stderr,"ERROR: out of bounds in spctrm_psd. Exiting.\n");
                      exit(0);
                  }
                  if (i>=2*m-1) {
                      fprintf(stderr,"ERROR: out of bounds in spctrm_psd. Exiting.\n");
                      exit(0);
                  }
                  psd[j][ii] = 2*(pow(w1[i],2) + pow(w1[i+1],2)); 
             }
        }

        for (i=0; i<m+1; i++) {
             p[i]=0;
             for (j=0; j<2*k; j++) {
                  p[i]+=psd[j][i];
             }
             p[i] /= (2*k*mm*sumw);
        }

}

// ********************************************************************************
// ********************************************************************************


#undef SWAP
#undef NRANSI

