/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "accel_calib.h"
#include "util.h"
#include "test_maths.h"

int test_direction();

int main(void){

    int r;

    // initialize random seed
    srand(time(NULL));

    debug("\nSTART testing maths",0);

    #ifdef DECIMATE 
        debug("\n      Start decimate test",0);
        r = test_decimate(2048,0.5,0.125);
        if (r) {
                debug("      *******************************\n      ***** Decimate test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Decimate test passed",0);
    #endif

    #ifdef INTEGRATION 
        debug("\n      Start integration test",0);
        r = test_integration(2048,1280,0.5);
        if (r) {
                debug("      *******************************\n      ***** Integration test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Integration test passed",0); 
    #endif

    #ifdef SAMPLE_LENGTH
        debug("\n      Start sample length test",0);
        r = test_sample_length();
        if (r) {
                debug("      *******************************\n      ***** Sample length test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("\n      Sample length test passed",0);
    #endif

    #ifdef DT
        debug("\n      Start dt test",0);
        r = test_dt();
        if (r) {
                debug("      *******************************\n      ***** dt test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("\n      dt test passed",0);
    #endif

    #ifdef SPECTRUM
        debug("\n      Start spectrum test",0);
        r = test_spectrum();
        if (r) {
                debug("      *******************************\n      ***** Spectrum test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("\n      Spectrum test passed",0);
    #endif

    #ifdef DETREND
        debug("\n      Start de-trend test",0);
        r = test_detrend();
        if (r) {
                debug("      *******************************\n      ***** Detrend test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Detrend test passed",0);
    #endif

    #ifdef DESPIKE
        debug("\n      Start despike test",0);
        r = test_despike();
        if (r) {
                debug("      *******************************\n      ***** Despike test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Despike test passed",0);
    #endif

    #ifdef DETERMINISTIC_ANALYSIS
        debug("\n      Start deterministic analysis test",0);
        r = test_deterministic_analysis(2048,1280,.5,2,256);
        if (r) {
                debug("      *******************************\n      ***** Deterministic test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Deterministic test passed",0);
    #endif

    #ifdef DISPLACEMENT
        debug("\n      Start displacment analysis test",0);
        r = test_displacement();
        if (r) {
                debug("      *******************************\n      ***** Displacement test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Displacement test passed",0);
    #endif

    #ifdef DIRECTION 
        debug("\n      Start direction test",0);
        r = test_direction();
        if (r) {
                debug("      *******************************\n      ***** Direction test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Direction test passed",0);
    #endif

    #ifdef RIG 
        debug("\n      Start rig test",0);
        r = test_rig();
        if (r) {
                debug("      *******************************\n      ***** Rig test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Rig test passed",0);
    #endif

    #ifdef TANK 
        debug("\n      Start wave tank test",0);
        r = test_tank();
        if (r) {
                debug("      *******************************\n      ***** Rig test FAILED ***** \n      *******************************",0); 
                exit(1);
        }
        else debug("      Wave tank test passed",0);
    #endif


    debug("\nEND   Testing maths\n",0);

    return r;
}
