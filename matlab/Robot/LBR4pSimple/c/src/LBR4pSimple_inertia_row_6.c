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



  I_row_6[0][0] = cos(q5)*sin(q2)*sin(q3)*3.818252E-1+cos(q2)*sin(q4)*sin(q5)*3.818252E-1+cos(q3)*cos(q4)*sin(q2)*sin(q5)*3.818252E-1+cos(q3)*cos(q6)*sin(q2)*sin(q5)*9.36E-3+cos(q5)*cos(q6)*sin(q2)*sin(q3)*9.126E-3+cos(q2)*cos(q6)*sin(q4)*sin(q5)*9.126E-3-sin(q2)*sin(q3)*sin(q4)*sin(q6)*9.36E-3+cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5)*9.126E-3+cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*9.36E-3;
  I_row_6[1][0] = cos(q3)*cos(q5)*3.818252E-1+cos(q3)*cos(q5)*cos(q6)*9.126E-3-cos(q4)*sin(q3)*sin(q5)*3.818252E-1-cos(q3)*sin(q4)*sin(q6)*9.36E-3-cos(q6)*sin(q3)*sin(q5)*9.36E-3+cos(q3)*cos(q4)*cos(q5)*cos(q6)*9.36E-3-cos(q4)*cos(q6)*sin(q3)*sin(q5)*9.126E-3;
  I_row_6[2][0] = sin(q4)*sin(q5)*(cos(q6)*2.2815E4+9.54563E5)*4.0E-7;
  I_row_6[3][0] = cos(q5)*(cos(q6)*2.2815E4+9.54563E5)*4.0E-7;
  I_row_6[5][0] = 3.818252E-1;
}
 

