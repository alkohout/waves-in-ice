/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include "util.h"

/* NOTE: Very simple string,float input is used to allow easy mapping to embedded systems */

void debug (char *in, float f) {
  if (f == 0)
    printf("%s\n", in);
  else
    printf("%s: %.20f\n", in, f);
}

void debug_LF (char *in, long double f) {
  if (f == 0)
    printf("%s\n", in);
  else
    printf("%s: %.20LF\n", in, f);
}

void debug_int (char *in, int f) {
  if (f == 0)
    printf("%s\n", in);
  else
    printf("%s: %d\n", in, f);
}

void logger (char *in, float f) {
  #ifdef LOGGER

  ido_l = fopen("log/process_data.log","a");
  if (ido_l == NULL) perror("Error process_data.log");
  if (f == 0) {
    fprintf(ido_l, "%s\n", in);
  }else{
    fprintf(ido_l, "%s: %f\n", in, f);
  }
  fclose(ido_l);

  #endif
}

void print2out(long double *data, int n, char *fname, FILE *id) {

    #ifdef PRINT_OUTPUT
    int i; 
   
    id = fopen(fname,"w");
	if (id == NULL)
		perror("Error");
    for (i=0; i<n; i++) fprintf(id,"%.20LF\n",data[i]); 
    fclose(id);
 
    #endif
}

void print2out_ld(long double *data, int n, char *fname, FILE *id) {

    int i; 

    id = fopen(fname,"w");
	if (id == NULL)
		perror("Error");
    for (i=0; i<n; i++) fprintf(id,"%LF10\n",data[i]); 
    fclose(id);
 
}

void print2out_2D(long double data[513][360], int n, int nn, char *fname, FILE *id) {

    int i,j; 

    id = fopen(fname,"w");
	if (id == NULL)
		perror("Error");
    for (i=0; i<n; i++) {
    for (j=0; j<nn; j++) {
            fprintf(id,"%LF\n",data[i][j]); 
    }
    }
    fclose(id);
 
}
