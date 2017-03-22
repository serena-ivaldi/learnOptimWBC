#include "LBR4pReal_inertia_row_6.h"

void LBR4pReal_inertia_row_6(double I_row_6[][1], const double* input1){
 
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



  I_row_6[0][0] = cos(q2)*cos(q4)*cos(q6)*(-7.0080888787E-5)+cos(q2)*cos(q4)*sin(q6)*2.46312607E-6+cos(q5)*sin(q2)*sin(q3)*9.134489670485E-3+cos(q2)*sin(q4)*sin(q5)*9.134489670485E-3+cos(q2)*cos(q5)*cos(q6)*sin(q4)*2.46312607E-6+cos(q3)*cos(q4)*sin(q2)*sin(q5)*9.134489670485E-3+cos(q3)*cos(q6)*sin(q2)*sin(q4)*7.0080888787E-5+cos(q3)*cos(q6)*sin(q2)*sin(q5)*1.351008E-2+cos(q5)*cos(q6)*sin(q2)*sin(q3)*1.3172328E-2+cos(q2)*cos(q5)*sin(q4)*sin(q6)*7.0080888787E-5+cos(q2)*cos(q6)*sin(q4)*sin(q5)*1.3172328E-2-cos(q3)*sin(q2)*sin(q4)*sin(q6)*2.46312607E-6+cos(q6)*sin(q2)*sin(q3)*sin(q4)*1.6576E-4+cos(q3)*sin(q2)*sin(q5)*sin(q6)*1.6576E-4+cos(q5)*sin(q2)*sin(q3)*sin(q6)*1.61616E-4-cos(q6)*sin(q2)*sin(q3)*sin(q5)*2.46312607E-6+cos(q2)*sin(q4)*sin(q5)*sin(q6)*1.61616E-4-sin(q2)*sin(q3)*sin(q4)*sin(q6)*1.351008E-2-sin(q2)*sin(q3)*sin(q5)*sin(q6)*7.0080888787E-5+cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*2.46312607E-6+cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*7.0080888787E-5+cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5)*1.3172328E-2+cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)*1.351008E-2+cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)*1.61616E-4+cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)*1.6576E-4;
  I_row_6[1][0] = -sin(q3)*(cos(q4)*sin(q5)*9.134489670485E-3+cos(q6)*sin(q4)*7.0080888787E-5+cos(q6)*sin(q5)*1.351008E-2-sin(q4)*sin(q6)*2.46312607E-6+sin(q5)*sin(q6)*1.6576E-4+cos(q4)*cos(q5)*cos(q6)*2.46312607E-6+cos(q4)*cos(q5)*sin(q6)*7.0080888787E-5+cos(q4)*cos(q6)*sin(q5)*1.3172328E-2+cos(q4)*sin(q5)*sin(q6)*1.61616E-4)+cos(q3)*(cos(q5)*1.185724550509452E44+cos(q5)*cos(q6)*1.709865932349759E44+cos(q6)*sin(q4)*2.151687818176833E42+cos(q5)*sin(q6)*2.097895622722412E42-cos(q6)*sin(q5)*3.197320438859059E40-sin(q4)*sin(q6)*1.753708648563855E44-sin(q5)*sin(q6)*9.097019467301719E41+cos(q4)*cos(q5)*cos(q6)*1.753708648563855E44+cos(q4)*cos(q5)*sin(q6)*2.151687818176833E42)*7.703719777548943E-47;
  I_row_6[2][0] = cos(q4)*cos(q6)*(-7.0080888787E-5)+cos(q4)*sin(q6)*2.46312607E-6+sin(q4)*sin(q5)*9.134489670485E-3+sin(q4)*sin(q5)*sin(q6)*1.61616E-4+cos(q5)*cos(q6)*sin(q4)*2.46312607E-6+cos(q5)*sin(q4)*sin(q6)*7.0080888787E-5+cos(q6)*sin(q4)*sin(q5)*1.3172328E-2;
  I_row_6[3][0] = cos(q5)*9.134489670485E-3+cos(q5)*cos(q6)*1.3172328E-2+cos(q5)*sin(q6)*1.61616E-4-cos(q6)*sin(q5)*2.46312607E-6-sin(q5)*sin(q6)*7.0080888787E-5;
  I_row_6[4][0] = cos(q6)*(-7.0080888787E-5)+sin(q6)*2.46312607E-6;
  I_row_6[5][0] = 9.134489670485E-3;
}
 

