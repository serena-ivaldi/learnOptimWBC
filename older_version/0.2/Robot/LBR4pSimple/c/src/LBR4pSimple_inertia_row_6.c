#include "LBR4pSimple_inertia_row_6.h"

void LBR4pSimple_inertia_row_6(double I_row_6[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
 
        I_row_6[0][0]=0;
 I_row_6[1][0]=0;
 I_row_6[2][0]=0;
 I_row_6[3][0]=0;
 I_row_6[4][0]=0;
 I_row_6[5][0]=0;
 I_row_6[6][0]=0;



  I_row_6[0][0] = cos(q5)*sin(q2)*sin(q3)*3.86912972E-1+cos(q2)*sin(q4)*sin(q5)*3.86912972E-1+cos(q3)*cos(q4)*sin(q2)*sin(q5)*3.86912972E-1+cos(q3)*cos(q6)*sin(q2)*sin(q5)*1.8216E-2+cos(q5)*cos(q6)*sin(q2)*sin(q3)*1.77606E-2+cos(q2)*cos(q6)*sin(q4)*sin(q5)*1.77606E-2-sin(q2)*sin(q3)*sin(q4)*sin(q6)*1.8216E-2+cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5)*1.77606E-2+cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*1.8216E-2;
  I_row_6[1][0] = cos(q3)*cos(q5)*3.86912972E-1+cos(q3)*cos(q5)*cos(q6)*1.77606E-2-cos(q4)*sin(q3)*sin(q5)*3.86912972E-1-cos(q3)*sin(q4)*sin(q6)*1.8216E-2-cos(q6)*sin(q3)*sin(q5)*1.8216E-2+cos(q3)*cos(q4)*cos(q5)*cos(q6)*1.8216E-2-cos(q4)*cos(q6)*sin(q3)*sin(q5)*1.77606E-2;
  I_row_6[2][0] = sin(q4)*sin(q5)*(cos(q6)*4.44015E6+9.6728243E7)*4.0E-9;
  I_row_6[3][0] = cos(q5)*(cos(q6)*4.44015E6+9.6728243E7)*4.0E-9;
  I_row_6[5][0] = 3.86912972E-1;
}
 

