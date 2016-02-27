#include "LBR4pSimple_inertia_row_4.h"

void LBR4pSimple_inertia_row_4(double I_row_4[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
 
        I_row_4[0][0]=0;
 I_row_4[1][0]=0;
 I_row_4[2][0]=0;
 I_row_4[3][0]=0;
 I_row_4[4][0]=0;
 I_row_4[5][0]=0;
 I_row_4[6][0]=0;



  I_row_4[0][0] = sin(q2)*sin(q3)*1.80756+pow(cos(q5),2.0)*sin(q2)*sin(q3)*1.8252E-3+pow(cos(q6),2.0)*sin(q2)*sin(q3)*1.8252E-3+cos(q4)*sin(q2)*sin(q3)*(3.51E2/6.25E2)+cos(q6)*sin(q2)*sin(q3)*1.8252E-2-pow(cos(q5),2.0)*pow(cos(q6),2.0)*sin(q2)*sin(q3)*1.8252E-3+cos(q4)*cos(q6)*sin(q2)*sin(q3)*9.36E-3+cos(q2)*cos(q5)*sin(q4)*sin(q5)*1.8252E-3-cos(q2)*cos(q4)*sin(q5)*sin(q6)*9.126E-3-cos(q2)*cos(q5)*pow(cos(q6),2.0)*sin(q4)*sin(q5)*1.8252E-3+cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q5)*1.8252E-3-cos(q2)*cos(q4)*cos(q6)*sin(q5)*sin(q6)*1.8252E-3+cos(q3)*sin(q2)*sin(q4)*sin(q5)*sin(q6)*9.126E-3-cos(q5)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*9.36E-3+cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q5)*sin(q6)*1.8252E-3-cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q2)*sin(q5)*1.8252E-3;
  I_row_4[1][0] = cos(q3)*1.80756+cos(q3)*cos(q4)*(3.51E2/6.25E2)+cos(q3)*cos(q6)*1.8252E-2+cos(q3)*pow(cos(q5),2.0)*1.8252E-3+cos(q3)*pow(cos(q6),2.0)*1.8252E-3-cos(q3)*pow(cos(q5),2.0)*pow(cos(q6),2.0)*1.8252E-3+cos(q3)*cos(q4)*cos(q6)*9.36E-3-cos(q4)*cos(q5)*sin(q3)*sin(q5)*1.8252E-3-cos(q3)*cos(q5)*sin(q4)*sin(q6)*9.36E-3-sin(q3)*sin(q4)*sin(q5)*sin(q6)*9.126E-3+cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q3)*sin(q5)*1.8252E-3-cos(q6)*sin(q3)*sin(q4)*sin(q5)*sin(q6)*1.8252E-3;
  I_row_4[2][0] = sin(q5*2.0)*sin(q4)*4.563E-4-sin(q6*2.0)*cos(q4)*sin(q5)*9.126E-4-cos(q6*2.0)*sin(q5*2.0)*sin(q4)*4.563E-4-cos(q4)*sin(q5)*sin(q6)*9.126E-3;
  I_row_4[3][0] = cos(q5*2.0)*9.126E-4+cos(q6)*1.8252E-2+pow(cos(q6),2.0)*9.126E-4-cos(q5*2.0)*pow(cos(q6),2.0)*9.126E-4+1.8084726;
  I_row_4[4][0] = sin(q5)*sin(q6)*(cos(q6)+5.0)*(-1.8252E-3);
  I_row_4[5][0] = cos(q5)*(cos(q6)*2.2815E4+9.54563E5)*4.0E-7;
  I_row_4[6][0] = sin(q5)*sin(q6)*(3.0/5.0E1);
}
 

