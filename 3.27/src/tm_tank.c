/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "util.h"
#include "main_test.h"
#include "process_data.h"
#include "signal_conditioning.h"

int test_tank(){

  FILE *idi, *ida;
  int i, r=0, n=155, na=1280;
  long double Ht, mean_accel, Tp;
  long double processed[n], accel[na];

  // Open processed file
  idi = fopen("out/processed.out","r");
  if (idi == NULL) perror("Error cannot open processed.out");
  for ( i=0; i<n; i++) fscanf(idi,"%LF",&processed[i]); 
  fclose(idi);
  Ht = processed[100];
  Tp = processed[16];
 
  debug("        Wave height",0);
  debug("          Tank Ht (m)             : 0.0319 ",0);
  debug("          Ht WII version 3.27     : 0.0319",0);
  debug("          Ht WII current version  ", Ht);
  debug("        Tp",0);
  debug("          Tank Tp (s)                  : 6.67 ",0);
  debug("          Tp (s) WII version 3.27      : 6.67",0);
  debug("          Tp WII current version       ", Tp);
  //debug("        ",0);
  //debug("        Perfect Acceleration (m) :              0.1017 ",0);
  //debug("        Mean acceleration WII version 3.27 :    0.1018",0);
  //debug("        WII current version  ",mean_accel);

  // Test if Hs is within bounds of SWIFT Hs from Arctic (ship based test - 815)
//  if ((0.10 < Hs) && (Hs < 0.12) ) r = 1;


  return(r);
}

