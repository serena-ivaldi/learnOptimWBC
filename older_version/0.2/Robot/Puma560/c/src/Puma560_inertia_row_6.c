#include "Puma560_inertia_row_6.h"

void Puma560_inertia_row_6(double I_row_6[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
 
       I_row_6[0][0]=0;
 I_row_6[1][0]=0;
 I_row_6[2][0]=0;
 I_row_6[3][0]=0;
 I_row_6[4][0]=0;
 I_row_6[5][0]=0;



  I_row_6[0][0] = cos(q2)*(cos(q3)*cos(q5)*4.0E-5-cos(q4)*sin(q3)*sin(q5)*4.0E-5)-sin(q2)*(cos(q5)*sin(q3)*4.0E-5+cos(q3)*cos(q4)*sin(q5)*4.0E-5);
  I_row_6[1][0] = sin(q4)*sin(q5)*4.0E-5;
  I_row_6[2][0] = sin(q4)*sin(q5)*4.0E-5;
  I_row_6[3][0] = cos(q5)*4.0E-5;
  I_row_6[5][0] = 1.94104505668E-1;
}
 

