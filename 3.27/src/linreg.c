#include <stdlib.h>
#include <math.h>                           /* math functions                */
#include "util.h"

FILE *ido_x;
FILE *ido_y;

    int linreg(int n, long double *x, long double *y, long double a, long double b, long double r)
    {
        int i;
        long double   sumx = 0.0;                        /* sum of x                      */
        long double   sumx2 = 0.0;                       /* sum of x**2                   */
        long double   sumxy = 0.0;                       /* sum of x * y                  */
        long double   sumy = 0.0;                        /* sum of y                      */
        long double   sumy2 = 0.0;                       /* sum of y**2                   */

       print2out(x,n,"out/x.out",ido_x);
       print2out(y,n,"out/y.out",ido_y);

       for (i=0;i<n;i++)   
          { 
          sumx  += x[i];       
          sumx2 += x[i]*x[i];  
          sumxy += x[i]*y[i];
          sumy  += y[i];      
          sumy2 += y[i]*y[i]; 
          } 

       long double denom = (n * sumx2 - (sumx*sumx));
       if (denom == 0) {
           // singular matrix. can't solve the problem.
           a = 0;
           b = 0;
           if (r) r = 0;
           return 1;
       }
       debug("sumx",sumx);
       debug("sumy",sumy);
       debug("sumxy",sumxy);

       a = (n * sumxy  -  sumx * sumy) / denom;
       debug("a",a);
       b = (sumy * sumx2  -  sumx * sumxy) / denom;
       r = (sumxy - sumx * sumy / n) / sqrt((sumx2 - (sumx*sumx)/n) * (sumy2 - (sumy*sumy)/n));

       return 0; 
    }
