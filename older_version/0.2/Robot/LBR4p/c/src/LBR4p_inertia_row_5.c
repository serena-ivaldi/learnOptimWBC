#include "LBR4p_inertia_row_5.h"

void LBR4p_inertia_row_5(double I_row_5[][1], const double* input1){
 
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



  I_row_5[0][0] = cos(q2)*cos(q4)*1.4125101773904E-2+sin(q2)*sin(q3)*sin(q5)*8.50488880221E-4-cos(q2)*cos(q4)*pow(cos(q6),2.0)*5.550945674023E-3-cos(q3)*cos(q5)*sin(q2)*6.7524E-4-cos(q2)*cos(q5)*sin(q4)*8.50488880221E-4-cos(q3)*sin(q2)*sin(q4)*1.4125101773904E-2+cos(q3)*sin(q2)*sin(q5)*2.214328E-2+cos(q5)*sin(q2)*sin(q3)*2.3612519017301E-2+cos(q2)*sin(q4)*sin(q5)*2.3612519017301E-2+cos(q2)*cos(q5)*pow(cos(q6),2.0)*sin(q4)*4.647067532E-6+cos(q3)*pow(cos(q6),2.0)*sin(q2)*sin(q4)*5.550945674023E-3-pow(cos(q6),2.0)*sin(q2)*sin(q3)*sin(q5)*4.647067532E-6-cos(q3)*cos(q4)*cos(q5)*sin(q2)*8.50488880221E-4-cos(q3)*cos(q5)*cos(q6)*sin(q2)*1.6576E-4-cos(q2)*cos(q5)*cos(q6)*sin(q4)*1.61616E-4+cos(q2)*cos(q4)*cos(q6)*sin(q6)*4.647067532E-6+cos(q3)*cos(q4)*sin(q2)*sin(q5)*2.3612519017301E-2+cos(q4)*cos(q5)*sin(q2)*sin(q3)*2.214328E-2+cos(q3)*cos(q5)*sin(q2)*sin(q6)*1.351008E-2-cos(q5)*cos(q6)*sin(q2)*sin(q3)*7.0080888787E-5+cos(q2)*cos(q5)*sin(q4)*sin(q6)*1.3172328E-2-cos(q2)*cos(q6)*sin(q4)*sin(q5)*7.0080888787E-5+cos(q4)*sin(q2)*sin(q3)*sin(q5)*6.7524E-4+cos(q5)*sin(q2)*sin(q3)*sin(q6)*2.46312607E-6+cos(q6)*sin(q2)*sin(q3)*sin(q5)*1.61616E-4+cos(q2)*sin(q4)*sin(q5)*sin(q6)*2.46312607E-6-sin(q2)*sin(q3)*sin(q5)*sin(q6)*1.3172328E-2-cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*1.61616E-4+cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*1.3172328E-2-cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5)*7.0080888787E-5+cos(q2)*cos(q5)*cos(q6)*sin(q4)*sin(q6)*5.550945674023E-3+cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)*2.46312607E-6+cos(q4)*cos(q6)*sin(q2)*sin(q3)*sin(q5)*1.6576E-4-cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q6)*4.647067532E-6-cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*1.351008E-2-cos(q6)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*5.550945674023E-3+cos(q3)*cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q2)*4.647067532E-6+cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q6)*5.550945674023E-3;
  I_row_5[1][0] = cos(q3)*cos(q5)*2.3612519017301E-2+cos(q3)*sin(q5)*8.50488880221E-4+cos(q5)*sin(q3)*6.7524E-4+sin(q3)*sin(q4)*1.4125101773904E-2-sin(q3)*sin(q5)*2.214328E-2-cos(q3)*pow(cos(q6),2.0)*sin(q5)*4.647067532E-6-pow(cos(q6),2.0)*sin(q3)*sin(q4)*5.550945674023E-3+cos(q3)*cos(q4)*cos(q5)*2.214328E-2-cos(q3)*cos(q5)*cos(q6)*7.0080888787E-5+cos(q3)*cos(q4)*sin(q5)*6.7524E-4+cos(q4)*cos(q5)*sin(q3)*8.50488880221E-4+cos(q3)*cos(q5)*sin(q6)*2.46312607E-6+cos(q3)*cos(q6)*sin(q5)*1.61616E-4+cos(q5)*cos(q6)*sin(q3)*1.6576E-4-cos(q4)*sin(q3)*sin(q5)*2.3612519017301E-2-cos(q3)*sin(q5)*sin(q6)*1.3172328E-2-cos(q5)*sin(q3)*sin(q6)*1.351008E-2-cos(q4)*cos(q5)*pow(cos(q6),2.0)*sin(q3)*4.647067532E-6+cos(q3)*cos(q4)*cos(q6)*sin(q5)*1.6576E-4+cos(q4)*cos(q5)*cos(q6)*sin(q3)*1.61616E-4-cos(q3)*cos(q4)*sin(q5)*sin(q6)*1.351008E-2-cos(q4)*cos(q5)*sin(q3)*sin(q6)*1.3172328E-2+cos(q4)*cos(q6)*sin(q3)*sin(q5)*7.0080888787E-5-cos(q3)*cos(q6)*sin(q5)*sin(q6)*5.550945674023E-3-cos(q4)*sin(q3)*sin(q5)*sin(q6)*2.46312607E-6+cos(q6)*sin(q3)*sin(q4)*sin(q6)*4.647067532E-6-cos(q4)*cos(q5)*cos(q6)*sin(q3)*sin(q6)*5.550945674023E-3;
  I_row_5[2][0] = cos(q4)*1.4125101773904E-2-cos(q5)*sin(q4)*8.50488880221E-4+sin(q4)*sin(q5)*2.3612519017301E-2-cos(q4)*pow(cos(q6),2.0)*5.550945674023E-3+sin(q4)*sin(q5)*sin(q6)*2.46312607E-6+cos(q5)*pow(cos(q6),2.0)*sin(q4)*4.647067532E-6-cos(q5)*cos(q6)*sin(q4)*1.61616E-4+cos(q4)*cos(q6)*sin(q6)*4.647067532E-6+cos(q5)*sin(q4)*sin(q6)*1.3172328E-2-cos(q6)*sin(q4)*sin(q5)*7.0080888787E-5+cos(q5)*cos(q6)*sin(q4)*sin(q6)*5.550945674023E-3;
  I_row_5[3][0] = cos(q5)*2.3612519017301E-2+sin(q5)*8.50488880221E-4-cos(q5)*cos(q6)*7.0080888787E-5+cos(q5)*sin(q6)*2.46312607E-6+cos(q6)*sin(q5)*1.61616E-4-sin(q5)*sin(q6)*1.3172328E-2-pow(cos(q6),2.0)*sin(q5)*4.647067532E-6-cos(q6)*sin(q5)*sin(q6)*5.550945674023E-3;
  I_row_5[4][0] = cos(q6*2.0)*(-2.7754728370115E-3)+sin(q6*2.0)*2.323533766E-6+1.820559205938925E-1;
  I_row_5[5][0] = cos(q6)*(-7.0080888787E-5)+sin(q6)*2.46312607E-6;
  I_row_5[6][0] = cos(q6)*1.584375E-4;
}
 

