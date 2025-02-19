/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "util.h"
#include "main_test.h"
#include "process_data.h"
#include "signal_conditioning.h"

int test_rig(){

  FILE *idi, *ida;
  int i, r=0, n=155, na=1280;
  long double mean_h, mean_accel, Tp;
  long double processed[n], accel[na];

  // Open processed file
  idi = fopen("out/processed.out","r");
  if (idi == NULL) perror("Error cannot open processed.out");
  for ( i=0; i<n; i++) fscanf(idi,"%LF",&processed[i]); 
  fclose(idi);
  mean_h = processed[100];
  Tp = processed[16];
 
  // Open acceleration data
  //ida = fopen("out/accel_2hz.out","r");
  //if (ida = NULL) perror("error cannont open processed.out");
  //for ( i=0; i<na; i++) fscanf(idi,"%LF",&accel[i]); 
  //fclose(ida);
  //mean_accel = 0.0;
  //for ( i=0; i<na; i++) mean_accel += accel[i]; 
  //mean_accel /= na;

  debug("        Height",0);
  debug("          Perfect Mean Height (m)          : 0.5 ",0);
  debug("          Mean height WII version 3.27     : 0.475",0);
  debug("          Mean height WII current version  ",   mean_h);
  debug("        Tp",0);
  debug("          Tp (s) WII version 3.27          : 9.85",0);
  debug("          WII current version              ", Tp);

  // Test if Hs is within bounds of rig test
  if ((0.46 > mean_h) || (mean_h > 0.52) ) r = 1;


  return(r);
}

