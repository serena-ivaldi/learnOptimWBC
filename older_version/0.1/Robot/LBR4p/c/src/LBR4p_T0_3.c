#include "LBR4p_T0_3.h"

void LBR4p_T0_3(double T[][4], const double* input1){
 
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



  T[0][0] = -sin(q1)*sin(q3)+cos(q1)*cos(q2)*cos(q3);
  T[0][1] = cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1);
  T[0][2] = -cos(q3)*sin(q2);
  T[1][0] = -cos(q1)*sin(q2);
  T[1][1] = -sin(q1)*sin(q2);
  T[1][2] = -cos(q2);
  T[2][0] = -cos(q3)*sin(q1)-cos(q1)*cos(q2)*sin(q3);
  T[2][1] = cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3);
  T[2][2] = sin(q2)*sin(q3);
  T[3][0] = cos(q1)*sin(q2)*(2.0/5.0);
  T[3][1] = sin(q1)*sin(q2)*(2.0/5.0);
  T[3][2] = cos(q2)*(2.0/5.0)+3.1E1/1.0E2;
  T[3][3] = 1.0;
}
 

