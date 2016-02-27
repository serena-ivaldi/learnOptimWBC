#include "LBR4pSimple_inertia_row_5.h"

void LBR4pSimple_inertia_row_5(double I_row_5[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
 
        I_row_5[0][0]=0;
 I_row_5[1][0]=0;
 I_row_5[2][0]=0;
 I_row_5[3][0]=0;
 I_row_5[4][0]=0;
 I_row_5[5][0]=0;
 I_row_5[6][0]=0;



  I_row_5[0][0] = cos(q2)*cos(q4)*7.218252E-1-cos(q2)*cos(q4)*pow(cos(q6),2.0)*1.8252E-3-cos(q3)*sin(q2)*sin(q4)*7.218252E-1+cos(q3)*pow(cos(q6),2.0)*sin(q2)*sin(q4)*1.8252E-3+cos(q3)*cos(q5)*sin(q2)*sin(q6)*9.36E-3+cos(q2)*cos(q5)*sin(q4)*sin(q6)*9.126E-3-sin(q2)*sin(q3)*sin(q5)*sin(q6)*9.126E-3+cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*9.126E-3+cos(q2)*cos(q5)*cos(q6)*sin(q4)*sin(q6)*1.8252E-3-cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*9.36E-3-cos(q6)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*1.8252E-3+cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q6)*1.8252E-3;
  I_row_5[1][0] = sin(q3)*sin(q4)*7.218252E-1-pow(cos(q6),2.0)*sin(q3)*sin(q4)*1.8252E-3-cos(q3)*sin(q5)*sin(q6)*9.126E-3-cos(q5)*sin(q3)*sin(q6)*9.36E-3-cos(q3)*cos(q4)*sin(q5)*sin(q6)*9.36E-3-cos(q4)*cos(q5)*sin(q3)*sin(q6)*9.126E-3-cos(q3)*cos(q6)*sin(q5)*sin(q6)*1.8252E-3-cos(q4)*cos(q5)*cos(q6)*sin(q3)*sin(q6)*1.8252E-3;
  I_row_5[2][0] = cos(q4)*7.218252E-1-cos(q4)*pow(cos(q6),2.0)*1.8252E-3+cos(q5)*sin(q4)*sin(q6)*9.126E-3+cos(q5)*cos(q6)*sin(q4)*sin(q6)*1.8252E-3;
  I_row_5[3][0] = sin(q5)*sin(q6)*(cos(q6)+5.0)*(-1.8252E-3);
  I_row_5[4][0] = cos(q6*2.0)*(-9.126E-4)+7.209126E-1;
  I_row_5[6][0] = cos(q6)*(3.0/5.0E1);
}
 

