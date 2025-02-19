/* vim: set tabstop=8 softtabstop=8 shiftwidth=8 expandtab : */


// PURPOSE: calculate wave direction functions
#include <math.h>
#include "util.h"
#include "main.h"
#include "signal_conditioning.h"
#include "rpy.h"

// ********************************************************************************
// ********************************************************************************

int rpy(double *gyro, double *accel, double *magno){

    //XXX AK Include testing

    int r=0;

    gyro_x = gyro[0];
    gyro_y = gyro[1];
    gyro_z = gyro[2];
    
    accel_x = accel[0];
    accel_y = accel[1];
    accel_z = accel[2];

    magnetom_x = magno[0];
    magnetom_y = magno[1];
    magnetom_z = magno[2];
  
    // Calculations...
    Compass_Heading();
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();

    return(r);

}

// ********************************************************************************
// ********************************************************************************
// Local magnetic declination
// I use this web : http://www.ngdc.noaa.gov/geomagmodels/Declination.jsp
#define MAGNETIC_DECLINATION -6.0    // not used now -> magnetic bearing

void Compass_Heading()
{
  double MAG_X;
  double MAG_Y;
  double cos_roll;
  double sin_roll;
  double cos_pitch;
  double sin_pitch;
  
  cos_roll = cos(roll_t);
  sin_roll = sin(roll_t);

  cos_pitch = cos(pitch_t);
  sin_pitch = sin(pitch_t);

  // Tilt compensated Magnetic filed X and Y
  MAG_X=magnetom_x*cos_pitch+magnetom_y*sin_roll*sin_pitch+magnetom_z*cos_roll*sin_pitch;
  MAG_Y=magnetom_y*cos_roll-magnetom_z*sin_roll;

  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
}

// ********************************************************************************
// ********************************************************************************

void Normalize(void)
{
  double error=0;
  double temporary[3][3];
  double renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

// ********************************************************************************
// ********************************************************************************

void Drift_correction(void)
{
  double mag_heading_x;
  double mag_heading_y;
  double errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static double Scaled_Omega_P[3];
  static double Scaled_Omega_I[3];
  double Accel_magnitude;
  double Accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)

  Accel_weight = 1;//constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  
  // Constrain funct not working, so
  /*
  if ( ((1-2*abs(1-Accel_magnitude)) < 0.5) || ((1-2*abs(1-Accel_magnitude)) > 1.5)) Accel_weight = 0;
  else Accel_weight = 1;
  */

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference

  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I


}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}

*/

// ********************************************************************************
// ********************************************************************************

void Matrix_update(void)
{

  int x,y;

  Gyro_Vector[0]=gyro_x; 
  Gyro_Vector[1]=gyro_y;
  Gyro_Vector[2]=gyro_z;
  
  Accel_Vector[0]=accel_x;
  Accel_Vector[1]=accel_y;
  Accel_Vector[2]=accel_z;
    
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
  
 #if OUTPUTMODE==1         
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(x=0; x<3; x++) //Matrix Addition (update)
  {
    for(y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

// ********************************************************************************
// ********************************************************************************

void Euler_angles(void)
{
  pitch_t = -asin(DCM_Matrix[2][0]);
  roll_t = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw_t = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

// ********************************************************************************
// ********************************************************************************

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(double a[3][3], double b[3][3],double mat[3][3])
{
  int x, y, w;
  double op[3]; 

  for(x=0; x<3; x++)
  {
    for(y=0; y<3; y++)
    {
      for(w=0; w<3; w++)
      {
       op[w]=a[x][w]*b[w][y];
      } 
      mat[x][y]=0;
      mat[x][y]=op[0]+op[1]+op[2];
      
      double test=mat[x][y];
    }
  }
}

// ********************************************************************************
// ********************************************************************************

//Computes the dot product of two vectors
double Vector_Dot_Product(double vector1[3],double vector2[3])
{
  int c;
  double op=0;
  
  for(c=0; c<3; c++)
  {
  op+=vector1[c]*vector2[c];
  }
  return op; 
}

// ********************************************************************************
// ********************************************************************************

//Computes the cross product of two vectors
void Vector_Cross_Product(double vectorOut[3], double v1[3],double v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

// ********************************************************************************
// ********************************************************************************

//Multiply the vector by a scalar. 
void Vector_Scale(double vectorOut[3],double vectorIn[3], double scale2)
{
  int c;

  for(c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2; 
  }
}

// ********************************************************************************
// ********************************************************************************

void Vector_Add(double vector_out[3], double vector_in_1[3], double vector_in_2[3]) {

       int c;

       for (c=0; c<3; c++){

              vector_out[c] = vector_in_1[c] + vector_in_2[c];
       }

}

