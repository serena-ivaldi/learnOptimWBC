#include "LBR4pSimple_inertia_row_3.h"

void LBR4pSimple_inertia_row_3(double I_row_3[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
 
        I_row_3[0][0]=0;
 I_row_3[1][0]=0;
 I_row_3[2][0]=0;
 I_row_3[3][0]=0;
 I_row_3[4][0]=0;
 I_row_3[5][0]=0;
 I_row_3[6][0]=0;



  I_row_3[0][0] = cos(q2)*2.354472972+cos(q2)*cos(q6)*3.55212E-2-cos(q2)*pow(cos(q4),2.0)*5.4756E-1-cos(q2)*pow(cos(q5),2.0)*6.912972E-3-cos(q2)*pow(cos(q4),2.0)*cos(q6)*3.55212E-2+cos(q2)*pow(cos(q4),2.0)*pow(cos(q5),2.0)*6.912972E-3-cos(q2)*pow(cos(q4),2.0)*pow(cos(q6),2.0)*6.912972E-3+cos(q2)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*6.912972E-3+cos(q3)*sin(q2)*sin(q4)*(3.51E2/6.25E2)-cos(q2)*pow(cos(q4),2.0)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*6.912972E-3+cos(q3)*cos(q4)*sin(q2)*sin(q4)*5.4756E-1+cos(q3)*cos(q6)*sin(q2)*sin(q4)*1.8216E-2-cos(q3)*cos(q5)*sin(q2)*sin(q6)*1.77606E-2-sin(q2)*sin(q3)*sin(q5)*sin(q6)*1.8216E-2-cos(q3)*cos(q4)*pow(cos(q5),2.0)*sin(q2)*sin(q4)*6.912972E-3+cos(q3)*cos(q4)*pow(cos(q6),2.0)*sin(q2)*sin(q4)*6.912972E-3+cos(q3)*pow(cos(q4),2.0)*cos(q5)*sin(q2)*sin(q6)*3.55212E-2+cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q4)*3.55212E-2+cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*1.8216E-2+cos(q2)*cos(q4)*cos(q5)*sin(q4)*sin(q6)*3.55212E-2-cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q6)*6.912972E-3+cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*6.912972E-3-cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*1.77606E-2+cos(q3)*cos(q4)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)*sin(q4)*6.912972E-3+cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q4)*sin(q6)*1.3825944E-2-cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*6.912972E-3+cos(q3)*pow(cos(q4),2.0)*cos(q5)*cos(q6)*sin(q2)*sin(q6)*1.3825944E-2-cos(q5)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*6.912972E-3;
  I_row_3[1][0] = sin(q3)*sin(q4)*(-3.51E2/6.25E2)-sin(q4*2.0)*sin(q3)*2.746441215E-1+sin(q5*2.0)*cos(q3)*sin(q4)*1.728243E-3-sin(q4*2.0)*cos(q6)*sin(q3)*1.77606E-2+cos(q5*2.0)*sin(q4*2.0)*sin(q3)*8.641215E-4-cos(q6*2.0)*sin(q4*2.0)*sin(q3)*2.5923645E-3-cos(q6)*sin(q3)*sin(q4)*1.8216E-2-cos(q3)*sin(q5)*sin(q6)*1.8216E-2-cos(q4*2.0)*cos(q5)*sin(q3)*sin(q6)*1.77606E-2-sin(q6*2.0)*cos(q3)*cos(q4)*sin(q5)*3.456486E-3-cos(q4*2.0)*sin(q6*2.0)*cos(q5)*sin(q3)*3.456486E-3-cos(q6*2.0)*sin(q5*2.0)*cos(q3)*sin(q4)*1.728243E-3-cos(q3)*cos(q4)*sin(q5)*sin(q6)*1.77606E-2-cos(q4)*cos(q5)*sin(q3)*sin(q6)*1.8216E-2-cos(q5*2.0)*cos(q6*2.0)*sin(q4*2.0)*sin(q3)*8.641215E-4;
  I_row_3[2][0] = cos(q4*2.0)*(-2.7378E-1)+cos(q6)*1.77606E-2-pow(cos(q5),2.0)*3.456486E-3-pow(cos(q6),2.0)*3.456486E-3-cos(q4*2.0)*cos(q6)*1.77606E-2+cos(q4*2.0)*pow(cos(q5),2.0)*3.456486E-3-cos(q4*2.0)*pow(cos(q6),2.0)*3.456486E-3+pow(cos(q5),2.0)*pow(cos(q6),2.0)*3.456486E-3+sin(q4*2.0)*cos(q5)*sin(q6)*1.77606E-2-cos(q4*2.0)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*3.456486E-3+sin(q4*2.0)*cos(q5)*cos(q6)*sin(q6)*6.912972E-3+2.080692972;
  I_row_3[3][0] = sin(q5*2.0)*sin(q4)*1.728243E-3-sin(q6*2.0)*cos(q4)*sin(q5)*3.456486E-3-cos(q6*2.0)*sin(q5*2.0)*sin(q4)*1.728243E-3-cos(q4)*sin(q5)*sin(q6)*1.77606E-2;
  I_row_3[4][0] = cos(q4)*7.26912972E-1-cos(q4)*pow(cos(q6),2.0)*6.912972E-3+cos(q5)*sin(q4)*sin(q6)*1.77606E-2+cos(q5)*cos(q6)*sin(q4)*sin(q6)*6.912972E-3;
  I_row_3[5][0] = sin(q4)*sin(q5)*(cos(q6)*4.44015E6+9.6728243E7)*4.0E-9;
  I_row_3[6][0] = cos(q4)*cos(q6)*(3.0/5.0E1)-cos(q5)*sin(q4)*sin(q6)*(3.0/5.0E1);
}
 

