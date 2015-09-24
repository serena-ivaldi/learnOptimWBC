#include "Puma560_inertia_row_1.h"

void Puma560_inertia_row_1(double I_row_1[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
 
       I_row_1[0][0]=0;
 I_row_1[1][0]=0;
 I_row_1[2][0]=0;
 I_row_1[3][0]=0;
 I_row_1[4][0]=0;
 I_row_1[5][0]=0;



  I_row_1[0][0] = sin(q2*2.0)*4.173999E-3+sin(q3*2.0)*1.1273199E-2-cos(q4)*sin(q5)*1.16928E-4+sin(q4)*sin(q5)*8.64288E-4+pow(cos(q2),2.0)*1.9257810475+pow(cos(q3),2.0)*3.239198455E-1-pow(cos(q5),2.0)*2.0216E-4+pow(cos(q2),2.0)*cos(q3)*2.191385E-2+pow(cos(q2),2.0)*cos(q5)*2.487168E-3+pow(cos(q3),2.0)*cos(q5)*2.487168E-3-pow(cos(q2),2.0)*sin(q3)*7.69752588E-1-pow(cos(q2),2.0)*pow(cos(q3),2.0)*6.47839691E-1-pow(cos(q2),2.0)*pow(cos(q4),2.0)*3.0216E-4+pow(cos(q2),2.0)*pow(cos(q5),2.0)*2.0216E-4-pow(cos(q3),2.0)*pow(cos(q4),2.0)*3.0216E-4+pow(cos(q3),2.0)*pow(cos(q5),2.0)*2.0216E-4-cos(q2)*pow(cos(q3),2.0)*sin(q2)*4.5092796E-2-pow(cos(q2),2.0)*cos(q3)*sin(q3)*4.5092796E-2-pow(cos(q2),2.0)*cos(q5)*sin(q3)*2.487168E-3+pow(cos(q2),2.0)*cos(q4)*sin(q5)*1.16928E-4+pow(cos(q3),2.0)*cos(q4)*sin(q5)*1.16928E-4-pow(cos(q2),2.0)*pow(cos(q3),2.0)*cos(q5)*4.974336E-3-cos(q2)*cos(q3)*sin(q2)*7.69752588E-1+cos(q2)*cos(q5)*sin(q2)*1.16928E-4+cos(q3)*cos(q5)*sin(q3)*1.16928E-4+pow(cos(q2),2.0)*pow(cos(q3),2.0)*pow(cos(q4),2.0)*6.0432E-4-pow(cos(q2),2.0)*pow(cos(q3),2.0)*pow(cos(q5),2.0)*4.0432E-4+pow(cos(q2),2.0)*pow(cos(q4),2.0)*pow(cos(q5),2.0)*2.0216E-4+pow(cos(q3),2.0)*pow(cos(q4),2.0)*pow(cos(q5),2.0)*2.0216E-4-cos(q2)*sin(q2)*sin(q3)*2.191385E-2-cos(q2)*pow(cos(q3),2.0)*cos(q5)*sin(q2)*2.33856E-4-pow(cos(q2),2.0)*cos(q3)*cos(q5)*sin(q3)*2.33856E-4-pow(cos(q2),2.0)*cos(q3)*cos(q4)*sin(q5)*2.487168E-3-pow(cos(q2),2.0)*pow(cos(q3),2.0)*pow(cos(q4),2.0)*pow(cos(q5),2.0)*4.0432E-4-pow(cos(q2),2.0)*pow(cos(q3),2.0)*cos(q4)*sin(q5)*2.33856E-4-cos(q2)*cos(q3)*cos(q5)*sin(q2)*2.487168E-3+cos(q2)*cos(q3)*sin(q2)*sin(q3)*6.47839691E-1-cos(q2)*cos(q4)*sin(q2)*sin(q5)*2.487168E-3-cos(q3)*cos(q4)*sin(q3)*sin(q5)*2.487168E-3-cos(q2)*cos(q3)*pow(cos(q4),2.0)*sin(q2)*sin(q3)*6.0432E-4+cos(q2)*cos(q3)*pow(cos(q5),2.0)*sin(q2)*sin(q3)*4.0432E-4+cos(q2)*pow(cos(q3),2.0)*cos(q4)*sin(q2)*sin(q5)*4.974336E-3+pow(cos(q2),2.0)*cos(q3)*cos(q4)*sin(q3)*sin(q5)*4.974336E-3+cos(q2)*cos(q3)*cos(q5)*sin(q2)*sin(q3)*4.974336E-3-cos(q2)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*4.0432E-4-cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*4.0432E-4+cos(q2)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*2.487168E-3+cos(q2)*cos(q3)*pow(cos(q4),2.0)*pow(cos(q5),2.0)*sin(q2)*sin(q3)*4.0432E-4+cos(q2)*cos(q3)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*2.33856E-4+cos(q2)*pow(cos(q3),2.0)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*8.0864E-4+pow(cos(q2),2.0)*cos(q3)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*8.0864E-4+2.337553212267;
  I_row_1[1][0] = cos(q2)*(-2.3751E-2)-sin(q2)*6.903913435E-1-cos(q2)*cos(q3)*1.384816665E-1-cos(q2)*sin(q3)*3.80751875E-3-cos(q3)*sin(q2)*3.80751875E-3+sin(q2)*sin(q3)*1.384816665E-1-sin(q2)*sin(q4)*sin(q5)*1.243584E-3-cos(q2)*cos(q3)*cos(q5)*4.32144E-4+cos(q5)*sin(q2)*sin(q3)*4.32144E-4+cos(q2)*cos(q4)*sin(q3)*sin(q4)*3.0216E-4+cos(q3)*cos(q4)*sin(q2)*sin(q4)*3.0216E-4-cos(q2)*cos(q3)*sin(q4)*sin(q5)*1.243584E-3+cos(q2)*cos(q4)*sin(q3)*sin(q5)*4.32144E-4+cos(q3)*cos(q4)*sin(q2)*sin(q5)*4.32144E-4-cos(q2)*sin(q3)*sin(q4)*sin(q5)*5.8464E-5-cos(q3)*sin(q2)*sin(q4)*sin(q5)*5.8464E-5+sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.243584E-3-cos(q2)*cos(q4)*pow(cos(q5),2.0)*sin(q3)*sin(q4)*2.0216E-4-cos(q3)*cos(q4)*pow(cos(q5),2.0)*sin(q2)*sin(q4)*2.0216E-4-cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5)*2.0216E-4+cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0216E-4;
  I_row_1[2][0] = cos(q2)*cos(q3)*(-1.384816665E-1)-cos(q2)*sin(q3)*3.80751875E-3-cos(q3)*sin(q2)*3.80751875E-3+sin(q2)*sin(q3)*1.384816665E-1-cos(q2)*cos(q3)*cos(q5)*4.32144E-4+cos(q5)*sin(q2)*sin(q3)*4.32144E-4+cos(q2)*cos(q4)*sin(q3)*sin(q4)*3.0216E-4+cos(q3)*cos(q4)*sin(q2)*sin(q4)*3.0216E-4-cos(q2)*cos(q3)*sin(q4)*sin(q5)*1.243584E-3+cos(q2)*cos(q4)*sin(q3)*sin(q5)*4.32144E-4+cos(q3)*cos(q4)*sin(q2)*sin(q5)*4.32144E-4-cos(q2)*sin(q3)*sin(q4)*sin(q5)*5.8464E-5-cos(q3)*sin(q2)*sin(q4)*sin(q5)*5.8464E-5+sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.243584E-3-cos(q2)*cos(q4)*pow(cos(q5),2.0)*sin(q3)*sin(q4)*2.0216E-4-cos(q3)*cos(q4)*pow(cos(q5),2.0)*sin(q2)*sin(q4)*2.0216E-4-cos(q2)*cos(q3)*cos(q5)*sin(q4)*sin(q5)*2.0216E-4+cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*2.0216E-4;
  I_row_1[3][0] = cos(q2)*cos(q3)*1.84216E-3-sin(q2)*sin(q3)*1.84216E-3-cos(q2)*cos(q3)*pow(cos(q5),2.0)*2.0216E-4+pow(cos(q5),2.0)*sin(q2)*sin(q3)*2.0216E-4-cos(q2)*cos(q4)*sin(q5)*1.243584E-3-cos(q2)*cos(q3)*cos(q4)*sin(q5)*5.8464E-5+cos(q2)*cos(q3)*sin(q4)*sin(q5)*4.32144E-4+cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.243584E-3+cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.243584E-3+cos(q4)*sin(q2)*sin(q3)*sin(q5)*5.8464E-5-sin(q2)*sin(q3)*sin(q4)*sin(q5)*4.32144E-4+cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q5)*2.0216E-4+cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*2.0216E-4;
  I_row_1[4][0] = cos(q2)*cos(q5)*sin(q4)*(-1.243584E-3)+cos(q2)*sin(q3)*sin(q4)*6.4216E-4+cos(q3)*sin(q2)*sin(q4)*6.4216E-4+cos(q2)*sin(q3)*sin(q5)*4.32144E-4+cos(q3)*sin(q2)*sin(q5)*4.32144E-4-cos(q2)*cos(q3)*cos(q4)*cos(q5)*4.32144E-4-cos(q2)*cos(q3)*cos(q5)*sin(q4)*5.8464E-5+cos(q2)*cos(q5)*sin(q3)*sin(q4)*1.243584E-3+cos(q3)*cos(q5)*sin(q2)*sin(q4)*1.243584E-3+cos(q4)*cos(q5)*sin(q2)*sin(q3)*4.32144E-4+cos(q5)*sin(q2)*sin(q3)*sin(q4)*5.8464E-5;
  I_row_1[5][0] = cos(q2+q3)*cos(q5)*4.0E-5-sin(q2+q3)*cos(q4)*sin(q5)*4.0E-5;
}
 

