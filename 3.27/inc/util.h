#ifndef util_h
#define util_h
#include <stdio.h>

FILE *ido_l;

// Simple debug - must provide a string and a float
void debug (char *in, float f);
void debug_LF (char *in, long double f);
void debug_int (char *in, int f);
void logger (char *in, float f);
void print2out(long double *data, int n, char *fname, FILE *id);
void print2out_ld(long double *data, int n, char *fname, FILE *id);
void print2out_2D(long double data[513][360], int n, int nn, char *fname, FILE *id);

#endif
