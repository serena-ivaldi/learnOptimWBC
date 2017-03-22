#include "LBR4pReal_T0_1.h"

void LBR4pReal_T0_1(double T[][4], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
 
                 T[0][0]=0;
 T[1][0]=0;
 T[2][0]=0;
 T[3][0]=0;
 T[0][1]=0;
 T[1][1]=0;
 T[2][1]=0;
 T[3][1]=0;
 T[0][2]=0;
 T[1][2]=0;
 T[2][2]=0;
 T[3][2]=0;
 T[0][3]=0;
 T[1][3]=0;
 T[2][3]=0;
 T[3][3]=0;



  T[0][0] = cos(q1);
  T[0][1] = sin(q1);
  T[1][2] = -1.0;
  T[2][0] = -sin(q1);
  T[2][1] = cos(q1);
  T[3][2] = 3.1E1/1.0E2;
  T[3][3] = 1.0;
}
 

