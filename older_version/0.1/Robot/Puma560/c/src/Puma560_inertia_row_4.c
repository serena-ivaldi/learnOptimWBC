#include "Puma560_inertia_row_4.h"

void Puma560_inertia_row_4(double I_row_4[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
 
       I_row_4[0][0]=0;
 I_row_4[1][0]=0;
 I_row_4[2][0]=0;
 I_row_4[3][0]=0;
 I_row_4[4][0]=0;
 I_row_4[5][0]=0;



  I_row_4[0][0] = cos(q2)*cos(q3)*1.84216E-3-sin(q2)*sin(q3)*1.84216E-3-cos(q2)*cos(q3)*pow(cos(q5),2.0)*2.0216E-4+pow(cos(q5),2.0)*sin(q2)*sin(q3)*2.0216E-4-cos(q2)*cos(q4)*sin(q5)*1.243584E-3-cos(q2)*cos(q3)*cos(q4)*sin(q5)*5.8464E-5+cos(q2)*cos(q3)*sin(q4)*sin(q5)*4.32144E-4+cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.243584E-3+cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.243584E-3+cos(q4)*sin(q2)*sin(q3)*sin(q5)*5.8464E-5-sin(q2)*sin(q3)*sin(q4)*sin(q5)*4.32144E-4+cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*2.0216E-4+cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*2.0216E-4;
  I_row_4[1][0] = sin(q4)*sin(q5)*(cos(q5)*1.2635E4-sin(q3)*7.7724E4+7.7724E4)*(-1.6E-8);
  I_row_4[2][0] = sin(q4)*sin(q5)*(cos(q5)*1.2635E4+7.7724E4)*(-1.6E-8);
  I_row_4[3][0] = cos(q5*2.0)*(-1.0108E-4)+1.9253170612368E-1;
  I_row_4[5][0] = cos(q5)*4.0E-5;
}
 

