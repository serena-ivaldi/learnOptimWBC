#include "Puma560_inertia_row_2.h"

void Puma560_inertia_row_2(double I_row_2[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
 
       I_row_2[0][0]=0;
 I_row_2[1][0]=0;
 I_row_2[2][0]=0;
 I_row_2[3][0]=0;
 I_row_2[4][0]=0;
 I_row_2[5][0]=0;



  I_row_2[0][0] = cos(q2)*(-2.3751E-2)-sin(q2)*6.903913435E-1-cos(q2)*cos(q3)*1.384816665E-1-cos(q2)*sin(q3)*3.80751875E-3-cos(q3)*sin(q2)*3.80751875E-3+sin(q2)*sin(q3)*1.384816665E-1-sin(q2)*sin(q4)*sin(q5)*1.243584E-3-cos(q2)*cos(q3)*cos(q5)*4.32144E-4+cos(q5)*sin(q2)*sin(q3)*4.32144E-4+cos(q2)*cos(q4)*sin(q3)*sin(q4)*3.0216E-4+cos(q3)*cos(q4)*sin(q2)*sin(q4)*3.0216E-4-cos(q2)*cos(q3)*sin(q4)*sin(q5)*1.243584E-3+cos(q2)*cos(q4)*sin(q3)*sin(q5)*4.32144E-4+cos(q3)*cos(q4)*sin(q2)*sin(q5)*4.32144E-4-cos(q2)*sin(q3)*sin(q4)*sin(q5)*5.8464E-5-cos(q3)*sin(q2)*sin(q4)*sin(q5)*5.8464E-5+sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.243584E-3-cos(q2)*cos(q4)*pow(cos(q5),2.0)*sin(q3)*sin(q4)*2.0216E-4-cos(q3)*cos(q4)*pow(cos(q5),2.0)*sin(q2)*sin(q4)*2.0216E-4-cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5)*2.0216E-4+cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0216E-4;
  I_row_2[1][0] = cos(q3)*2.191385E-2+cos(q5)*2.487168E-3-sin(q3)*7.69752588E-1-cos(q5)*sin(q3)*2.487168E-3-cos(q4)*sin(q5)*1.16928E-4+pow(cos(q4),2.0)*3.0216E-4+pow(cos(q5),2.0)*2.0216E-4-pow(cos(q4),2.0)*pow(cos(q5),2.0)*2.0216E-4-cos(q3)*cos(q4)*sin(q5)*2.487168E-3+4.4319189175;
  I_row_2[2][0] = cos(q3)*1.0956925E-2+cos(q5)*2.487168E-3-sin(q3)*3.84876294E-1-cos(q5)*sin(q3)*1.243584E-3-cos(q4)*sin(q5)*1.16928E-4+pow(cos(q4),2.0)*3.0216E-4+pow(cos(q5),2.0)*2.0216E-4-pow(cos(q4),2.0)*pow(cos(q5),2.0)*2.0216E-4-cos(q3)*cos(q4)*sin(q5)*1.243584E-3+3.589900705E-1;
  I_row_2[3][0] = sin(q4)*sin(q5)*(cos(q5)*1.2635E4-sin(q3)*7.7724E4+7.7724E4)*(-1.6E-8);
  I_row_2[4][0] = cos(q4)*6.4216E-4-sin(q5)*5.8464E-5+cos(q4)*cos(q5)*1.243584E-3-cos(q3)*sin(q5)*1.243584E-3-cos(q4)*cos(q5)*sin(q3)*1.243584E-3;
  I_row_2[5][0] = sin(q4)*sin(q5)*4.0E-5;
}
 

