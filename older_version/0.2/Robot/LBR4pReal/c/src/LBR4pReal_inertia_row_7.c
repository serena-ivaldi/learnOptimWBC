#include "LBR4pReal_inertia_row_7.h"

void LBR4pReal_inertia_row_7(double I_row_7[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
 
        I_row_7[0][0]=0;
 I_row_7[1][0]=0;
 I_row_7[2][0]=0;
 I_row_7[3][0]=0;
 I_row_7[4][0]=0;
 I_row_7[5][0]=0;
 I_row_7[6][0]=0;



  I_row_7[0][0] = cos(q2)*(cos(q4)*cos(q6)*1.584375E-4-cos(q5)*sin(q4)*sin(q6)*1.584375E-4)-sin(q2)*(cos(q3)*(cos(q6)*sin(q4)*1.584375E-4+cos(q4)*cos(q5)*sin(q6)*1.584375E-4)-sin(q3)*sin(q5)*sin(q6)*1.584375E-4);
  I_row_7[1][0] = sin(q3)*(cos(q6)*sin(q4)*1.584375E-4+cos(q4)*cos(q5)*sin(q6)*1.584375E-4)+cos(q3)*sin(q5)*sin(q6)*1.584375E-4;
  I_row_7[2][0] = cos(q4)*cos(q6)*1.584375E-4-cos(q5)*sin(q4)*sin(q6)*1.584375E-4;
  I_row_7[3][0] = sin(q5)*sin(q6)*1.584375E-4;
  I_row_7[4][0] = cos(q6)*1.584375E-4;
  I_row_7[6][0] = 1.584375E-4;
}
 

