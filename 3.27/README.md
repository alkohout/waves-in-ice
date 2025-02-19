
*********************************************************************************************************
UPDATING WII3_Maths.git
********************************************************************************************************* 
From new version
- cp all files from src and inc into WII3_Maths
- cp README.md into WII_Maths

cd WII3_Maths

git commit -am 'All my current changes' (make all your code safe and committed into the repository but only locally.)

git status (See if there are files you need to add, ie, new ones)
- git add FileName1 FileName2
- git commit -am 'New files'

git pull
- any merges should happen automatically, if not conflicts will need to be manually edited

git push

*********************************************************************************************************
V 3.27 
*********************************************************************************************************
20221101 - removing need to include declination define
- in sc_misc.c in mean_rpy add ifdef ARCTIC and ifdef DECLINATION
- It is now okay to remove declination define from makefile

20220928 - Fixing bug in Hs test
- in Makefile_test_Hs commented out DEFINE ID_SPECT_BAND
- Changed Hs test to height test on rig. Could no longer get the Arctic Swift comparison to work. Need to investigate why. 

20220928 - Fixed ratio bug
- in direction.c
  in wave_direction_FEM_2hz_accel initiated ui (ui=0)
- tm_direction.c
  in test_direction increased acceptable ratio range from (0.9 - 1.01) to (0.5 - 1.5)
   
20220021 - Segmentation faults fixed in run_test
- in tm_cycle.c
  in cycle_decimate 	- added malloc allocations to in, out, in_noise, out_noise 
  				- added free in, out, in_noise and out_noise
  in test_integreate  - added malloc allocations to data and disp
                      - added free data and disp
  				
20220823 - Edited sample size test
- in tm_misc.c
  in test_sample length() changed i upper limit from 13 to 11. 12 was causing seg fault in decimate test. Never discovered why.

20220811 - Comment out define DID_SPECT_BAND
- in Makefile_run
  comment out the line $(PROGSDIR)/$(VERSION): DEFINES+=-DID_SPECT_BAND so that the low frequency cut-off is fixed at 25 s

20220128 - Changed output from mean_yaw to std_yaw
- in process.c
  changed return_data->mean_yaw to return_data->std_yaw
  
20200817 - FIXED BANDS USED TO CALCULATE NOISE SLOPE
- in pd_misc.c
        - changed stt1 = 3 and ett1 = 10 (low frequency band from 85.33333 s - 28.44444 s - inclusive)
        - changed stt2 = 80 and ett2 = 129 (high frequency band from 3.2 s - 2 s - inclusive)

*********************************************************************************************************
V 3.26 - JARE61 2019 & NZ Navy 2022
*********************************************************************************************************
20191118 - FIXED HZ_MEAN DEFINITION
- in pd_misc.c
        - added if statment to definition to hz_mean to exclude first reading of hz (which is too large)

20190804 - ADDED IFDEF WAVETANK TO REMOVE MAGNETOMETER READINGS
- in direction.c
        - allocated yaw = 0 in north_east function if WAVETANK defined
        - allocated qflg_head = 4
- in Makefile_run 
        - added WAVETANK definition

20190803 - CORRECTED CALCULATION OF MEAN HEIGHT
- in pd_misc.c
	- moved calculation of hz_mean to within zero-crossing if statement so only calculates when final Hcm and Htm are calculated
	
20181019 - RUN_TEST WORKING WITH NO ERRORS

20181019 - CORRECTED MAKEFILE_TEST_HS 
- In Makefile_test_Hs
        - commented out define SAMPLE_64, uncommented SAMPLE_8 and ARCTIC defines

20181019 - CORRECT HEADERS IN TEST_MATHS TO ENSURE TEST_DIRECTION RUN
- In test_maths.h
        - remove structures

20181019 - TEST LIMITS OF VARIABLES USING TEST_MATHS
- In main.h
        - added limits to dt and n_accelfft

20181019 - ADDED HZ_MEAN TO DETERMINISTIC ANALYSIS IN TEST_MATHS
- In tm_deterministic_analysis.c
        - added hz_mean to call to deterministic function
- In test_maths.h
        - added hz_mean to retdata and initialisation to deterministic_analysis function

20181012 - IMPROVED HOW ZEROS ARE ADDED PRIOR TO INTEGRATION
- In process.c
        - removed method of simply adding zeros to end of data->data_z
        - added a 10% cosine taper to data->data_z
        - added zeros to beginning and end of data_accel_zeros, with data->data_z inbetween

- In tm_misc.c
        - applied %10 cosine window to period func output
        - added zerios to beginning and end of output to make size of data a power of 2

20180406 - REPLACED YAW MEAN OUTPUT WITH YAW STANDARD DEVIATION
- In process.c
        - replaced output return_data->yaw_mean with return_data->std_yaw*180.0/pi

20170525 - ADDED MEAN HEIGHT TO OUTPUT
- In pd_misc.c
        - added hz_mean definition to deterministic_analysis
        - added hz_mean to deterministic_analysis subroutine
- In process.c
        - added return_data->hz_mean to deterministic_analysis declaration 
        - added return_data->hz_mean to deterministic_analysis call
        - added return_data->hz mean to processed.out
- In main.h
        - added height_mean to RETDATA structure
- In process_data.h
        - added hz_mean to deterministic_analysis subroutine

20170426 - FIX MEAN YAW ERROR
- In signal_conditioning.c
        - added 2pi to calculation of mean_yaw and mean_yaw_sq to correct errors associated with calculation of mean yaw with negative values.
- In Makefile
        - changed define std_yaw_limit to 90.0. Still need to test what value should be. Possibly 0. 

*********************************************************************************************************
V 3.25 - PIPERS FINAL VERSION
*********************************************************************************************************

20170406 - MOVE DEFINES TO MAKEFILE
- In main.h
        - remove all defaults other than m, k and ardunio
- In Makefile
        - Add all defaults from main.h other than m, k and arduino
20170402 - UPDATES TO DIRECTIONAL ANALYSIS
- In direction.c
        - divided by omega^2 or 1 rather than omega^4 as appriopriate 
        - added kd (From doble)
        - added aa1s and bb1s from doble to calculate spread
        - divided c22 and c33 by kd to get a sensible ratio (logically should be kd^2, but answers not correct - seemingly).
20170331 - MOVED PRINT DIRECTION TO PROCESSSED.OUT
- In process.c
        - Added direction spectra to processed.out (if SEND_DIRECTION defined)
- In direction.c
        - allocated direction_full
- In main.h
        - added to return_data struct direction_full
20170331 - ADDED DEFINE NOISE FACTOR
- In pd_misc.c
        - replaced 0.2 with noise_factor in function identify_low_frequency_noise
- In main.h
        - added define noise_factor
20170331 - ADDED SEND_DIRECTION
- In Makefile
        - added DEFINE SEND_DIRECTION
20170327 - ADDED UI TO DIRECTION
- In direction.c
        - added ui to wave_direction_FEM_2hz_accel, replaced ratio with ui
        - changed spread function to be consistant with tucker_pitt eqn 7.2-13a. Also conistant with Doble.
20170327 - CLEANING
- In direction.c
        - removed wave_direction_FEM_2hz
20170327 - CHANGED OPEN WATER LIMIT
- In main.h
        - changed std_yaw_limit from 40 to 100 and then to 1   
20170327 - COMMENT OUT READ META.IN AND COMPASS.LOG (WHILE BUOY A'S ARE BROKEN)
- In main.c 
        - commented out read meta.in and compass.log
        - added data_raw.gps_heading=-9999.99
20170324 - UPDATE TEST DIRECTION
- In tm_direction.c
        - converted from wave_direction_FEM_2hz to wave_direction_FEM_2hz_accel
        - replaced include test_math.h with main.h
        - added functions definitions: period_func, north_east and wave_direction_2hz_accel
        - removed pi definition

20170321 - UPDATE HEADING DEFINITION 
- In Makefile_run
        - deleted DEFINE DRIG (obsolete)
- In main.c 
        - changed read file from gps_heading to compass.log
        - added read in meta file (start time and GPS instaneous heading)
        - added read compass.log and fill data_raw.gps_time and gps_headings
        - added return.n_gps (no. of gps heading readings)
        - added if != NULL compass.log, so program runs when no compass file
- In signal_condition.c
        - deleted ifdef RIG statment (obsolete)
        - added call to gpsraw2gps8hz
- In qc.c
        - changed quality_control_heading to read data_raw->gps_headings rather than data_raw->gps_headings 
        - in quality_control_heading replace n_raw with return_data->n_gps
- In direction.c
        - changed call from gps_heading to gps_headings in north_east
        - updated north_east to read gps_headings_8hz,  gps_heading, magnetometer at each time step or averaged.
- In sc_misc.c
        - added gpsraw_2_gps8hz
- In main.h
        - added idi_meta
        - added data_2hz.gps_headings
        - added data_raw.start 
        - added data_raw.gps_time[n_raw]
        - added data_raw.gps_headings[n_raw]
        - changed data_raw.gps_heading from 64Hz size to scalar
        - added return.n_gps (no. of gps heading readings)
        - added data_raw.gps_headings_8hz[n_raw]
        - added FILE *ido_gps
- In signal_conditioning.h
        - added gpsraw_2_gps8hz

20170320 - UPDATE KISTLER FLAG
- In qc.c
        - Added return_data->qflg_kistler in quality_control_accel

20170316 - UPDATE ERROR DEFINITION IN NOISE SLOPE
- In pd_misc.c
        - redefined return_data->r value determining return value in identify_low_frequency_noise 

20170310 - DIRECTIONAL ANALAYSIS VIA ACCELERATION RATHER THAN HEAVE
- In process.c
        - removed call to wave_direction_FEM_2hz
        - added call to wave_direction_FEM_2hz_accel 
- In direction.c
        - added wave_direction_FEM_2hz_accel
        - in wave_direction_FEM_2hz_accel
                - added omega multiplier
                - only calculated direction beetwen bounds f7 and f8
- In process.h
        - added wave_direction_FEM_2hz_accel

20170310 - FURTHER DEVELOPMENTS TO DIRECTIONAL ANALYSIS
- In direction.c
        - edited definition of Hs via C11
- In signal_conditioning.c
        - added output file accel_2hz
- In main.h
        - added FILE ido_a2

DEVELOPMENTS TO DIRECTIONAL ANALYSIS
- In direction.c
        - included variables st and ed which define the bounds for the direction analaysis
        - added coefficient to ratio calculation
        - added to wave direction Hs to return_data structure 
- In process.c
        - Added hs_dir to processed output
- In main.h
        - included rspns_f7 and rspns_f8 (bounds for directional analysis)
        - added hs_dir to return_data structure

*********************************************************************************************************
V 3.24
*********************************************************************************************************

20170131

DESPIKE LOOPING FOR KISTLER
- In qc.c
        - added a variable for the standard deviation mulitplier to despike to allow a different despike definition for Kistler versus IMU
- In qc.c
        - Added a do loop to repeat despike to ensure all kistler spikes removed
- In signal_conditioning.h
        - added the extra standard deviation variable to despike
- In main.h
        - added defines nstd_kist, nstd_imu, nstd_gyro, nstd_magno, nstd_pos

NORTH EAST BUG FIX
- In direction.c
        - fixed a bug in north_east. Roll calculation missing cos(yaw).

*********************************************************************************************************
V 3.23
*********************************************************************************************************
This version moves from reading in 40 min segment to reading 10min bursts.

20170125

OUTPUT
- In process.c
        - removed -90degrees from direction output

20170124

OPEN WATER FLAG
- In direction.c 
        - added an open_water definition of yaw in north_east
- In signal_conditioning.c
        - initialised return_data->open_water
        - defined return_data->open_water by the amount of standard deviation in yaw
- In process.c
        - Added open water flag to output
- In main.h
        - added return_data->qflg_open_water in structure
        - added define open_water

20170123

NORTH EAST ROLL AND PITCH
- In direction.c 
        - changed the calculation of roll and pitch in north_east. Now using the same calculation as Doble. 
        - changed north_east, so using return_data->mean_yaw, not data_raw->yaw_mean
        - added an open_water definition of yaw in north_east
- In main.h
        - removed data_raw->yaw_mean from structure.

20170117

TRAILING ZEROS TO MINISMISE SAMPLE TIME
- In tm_misc.c added trailing zeros to test_integration to test integration with zeros
- In process.c changed f_avg outputs to ensure good spread given new sample size

20170116

MOVING TO SHORTER SAMPLE SIZE
m = 256, k = 2
Tried
n_accel = 1280 (640 s / 10.66 mins), which after integration reduces to 1024, 512 secs / 8.53 mins.
- In direction.c
        - Changed w = 512 to w = m in wave_direction_FEM_2hz
- In process.c
        - Added zeros to data->accel_z to fill to sample length of a power of 2, so could call realft in intgrt_press
- In math.h 
        - Added define n_accelfft which is the next power of 2 of n_accel
        - Added data_2hz->accel_zeros, which is accel_z filled with zeros

*********************************************************************************************************
V 3.22
*********************************************************************************************************

20170116

DESPIKE
- in qc.c
        - Found two bugs in despike. Count was not initiated to 0 and the 'if (count > max)' statement was wrongly placed outside the 'if (fabsl ...)' statement.

TESTING
- update of testing_misc complete

20170112

INTEGRATION WITH N_SPEC VARIABLE
- In pd_misc.c
        - Added n_s in intgrt_press to enable variable n_spec, so that test_maths will run

- in process.c
        - Added n_s for each call to intgrt_press

- In process_data.h
        - Edited intgrt_press


20170111

GPS HEADING
- In main.c:
        - Added code to read GPS heading data

- In qc.c:
        - Added quality_control_heading

- In signal_condition.c
        - Initialised return_data->qflg_head

- In direction.c
        - Added gps heading to north_east

- In process.c
        - Added qflg_head to processed.out

- In signal_condition.h:
        - Added quality_control_heading

- In main.h:
        - Added qflg_head



20170110
SEND RAW
- In makefile: 
	- Added new define: SEND_RAW

- In signal_conditioning.c:
	- Added raw.out output (if SEND_RAW defined)

- In main.inc:
	- Added FILE *ido_raw;

IDENTIFY LOW FREQUENCY NOISE
- In pd_misc.c:
	- Added a function, identify_low_frequency_noise. This function fits a curve to the low frequency noise, finds its standard deviation and then the lowest frequency above this noise. This is then flagged as the lower edge of the frequency band with significant signal.

- In process.c:
	- Added, if ID_SPECT_BAND is defined, find the spectral band and calculate the intregal using responses defined by this band. If ID_SPECT_BAND is not defined, calculate the integral using the modifyable (in main.inc) rspns_f3 and rspns_f4.
        - Added to processed.out a (noise gradient), b (noise intercept), nstd (standard deviation of low frequency noise), response funtion cut off (in Hz).

- In Makefile:
        - Added $(PROGSDIR)/$(VERSION): DEFINES+=-DID_SPECT_BAND

- Standard sequence of tests now running (still plenty of work to do though)

FLAGS
- In process.c:
        - Added qflag_kist to processed.out
        - Added qflag_pkist to processed.out

- In qc.c:
        - Added return_data->qflg_kist to: quality_control_accel, 
        - Added return_data->qflg_pkist to: quality_control_accel, 
        - Changed void test_stats to int test_stats
        - Added function quality_control_rollpitch with flags
        - Defined return_data->qflg_imu in quality_control

- In signal_conditioning.c
        - Initialised: return_data->qflg_kist, return_data->qflg_accel,return_data->qflg_rollpitch, return_data->qflg_imu 
        - Added if statement which defines vertical acceleration depending on kistler and IMU qflgs.

- In signal_condition.h:
        - Changed void test_stats to int test_stats
        - Added void quality_control_rollpitch

- In main.h:
        - Added qflag_kist, qflag_pkist, qflag_imu, qflag_rollpitch

VERTICAL ACCEL
- In sc_misc.c:
        - Added ifdef Arctic in accel_orient_exact_imu
        - Removed accel_orient_exact_arduino

- In signal_condition.c:
        - Removed ifdef ARCTIC 
        
- In signal_condition.h:
        - Removed accel_orient_exact_arduino

LOGGERS
- In process.c:
        - Added extra logs

- In pd_misc.c:
        - Added extra logs
        
*********************************************************************************************************
V 3.21
*********************************************************************************************************

20160920
- Changed declination from -17 to +17

20161027
- added ifdef Arctic in accel_orient_exact_kist in sc_misc.c so arduino included to correct Arctic roll/pitch readings.

