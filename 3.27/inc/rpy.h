#ifndef rpy_h
#define rpy_h

void Matrix_update(void);
void Vector_Add(double vectorOut[3],double vectorIn1[3], double vectorIn2[3]);
void Matrix_Multiply(double a[3][3], double b[3][3],double mat[3][3]);
void Normalize(void);
void Compass_Heading(void);
void Drift_correction(void);
void Euler_angles(void);
double Vector_Dot_Product(double vector1[3],double vector2[3]);
void Vector_Cross_Product(double vectorOut[3], double v1[3],double v2[3]);
void Vector_Scale(double vectorOut[3],double vectorIn[3], double scale2);
void Vector_Add(double vector_out[3], double vector_in_1[3], double vector_in_2[3]);

#define Kp_ROLLPITCH 0.01
#define Ki_ROLLPITCH 0.00001
#define Kp_YAW 1.2
#define Ki_YAW 0.00002
#define GRAVITY g  // = 1G in the raw data coming from the accelerometer

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

double G_Dt=0.125;    // Integration time (DCM algorithm)

int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors; gyro accel

double accel_x;
double accel_y;
double accel_z;
double gyro_x;
double gyro_y;
double gyro_z;
double magnetom_x;
double magnetom_y;
double magnetom_z;
double MAG_Heading;

double Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
double Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
double Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
double Omega_P[3]= {0,0,0};//Omega Proportional correction
double Omega_I[3]= {0,0,0};//Omega Integrator
double Omega[3]= {0,0,0};

double errorRollPitch[3]= {0,0,0}; 
double errorYaw[3]= {0,0,0};

unsigned int counter=0;

double DCM_Matrix[3][3]= { { 1,0,0  } ,{ 0,1,0  } ,{ 0,0,1  } }; 
double Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
double Temporary_Matrix[3][3]={ { 0,0,0  } ,{ 0,0,0  } ,{ 0,0,0  } };

#endif
