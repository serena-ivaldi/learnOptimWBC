#include "Puma560_inertia_row_5.h"

void Puma560_inertia_row_5(double I_row_5[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
 
       I_row_5[0][0]=0;
 I_row_5[1][0]=0;
 I_row_5[2][0]=0;
 I_row_5[3][0]=0;
 I_row_5[4][0]=0;
 I_row_5[5][0]=0;



  I_row_5[0][0] = cos(q2)*cos(q5)*sin(q4)*(-1.243584E-3)+cos(q2)*sin(q3)*sin(q4)*6.4216E-4+cos(q3)*sin(q2)*sin(q4)*6.4216E-4+cos(q2)*sin(q3)*sin(q5)*4.32144E-4+cos(q3)*sin(q2)*sin(q5)*4.32144E-4-cos(q2)*cos(q3)*cos(q4)*cos(q5)*4.32144E-4-cos(q2)*cos(q3)*cos(q5)*sin(q4)*5.8464E-5+cos(q2)*cos(q5)*sin(q3)*sin(q4)*1.243584E-3+cos(q3)*cos(q5)*sin(q2)*sin(q4)*1.243584E-3+cos(q4)*cos(q5)*sin(q2)*sin(q3)*4.32144E-4+cos(q5)*sin(q2)*sin(q3)*sin(q4)*5.8464E-5;
  I_row_5[1][0] = cos(q4)*6.4216E-4-sin(q5)*5.8464E-5+cos(q4)*cos(q5)*1.243584E-3-cos(q3)*sin(q5)*1.243584E-3-cos(q4)*cos(q5)*sin(q3)*1.243584E-3;
  I_row_5[2][0] = cos(q4)*6.4216E-4-sin(q5)*5.8464E-5+cos(q4)*cos(q5)*1.243584E-3;
  I_row_5[4][0] = 1.71348451657E-1;
}
 

