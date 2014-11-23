#include "LBR4p_coriolis_row_7.h"

void LBR4p_coriolis_row_7(double C_row_7[][1], const double* input1, const double* input2){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
  double qd1 = input2[0];
  double qd2 = input2[1];
  double qd3 = input2[2];
  double qd4 = input2[3];
  double qd5 = input2[4];
  double qd6 = input2[5];
  double qd7 = input2[6];
 
        C_row_7[0][0]=0;
 C_row_7[1][0]=0;
 C_row_7[2][0]=0;
 C_row_7[3][0]=0;
 C_row_7[4][0]=0;
 C_row_7[5][0]=0;
 C_row_7[6][0]=0;



  C_row_7[0][0] = qd2*(cos(q4)*cos(q6)*sin(q2)*1.584375E-4+cos(q2)*cos(q3)*cos(q6)*sin(q4)*1.584375E-4-cos(q2)*sin(q3)*sin(q5)*sin(q6)*1.584375E-4-cos(q5)*sin(q2)*sin(q4)*sin(q6)*1.584375E-4+cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*1.584375E-4)*(-1.0/2.0)-qd6*(cos(q2)*cos(q4)*sin(q6)*1.584375E-4+cos(q2)*cos(q5)*cos(q6)*sin(q4)*1.584375E-4-cos(q3)*sin(q2)*sin(q4)*sin(q6)*1.584375E-4-cos(q6)*sin(q2)*sin(q3)*sin(q5)*1.584375E-4+cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*1.584375E-4)*(1.0/2.0)-qd4*(cos(q2)*cos(q6)*sin(q4)*1.584375E-4+cos(q3)*cos(q4)*cos(q6)*sin(q2)*1.584375E-4+cos(q2)*cos(q4)*cos(q5)*sin(q6)*1.584375E-4-cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6)*1.584375E-4)*(1.0/2.0)+qd5*sin(q6)*(cos(q5)*sin(q2)*sin(q3)+cos(q2)*sin(q4)*sin(q5)+cos(q3)*cos(q4)*sin(q2)*sin(q5))*7.921875E-5+qd3*sin(q2)*(cos(q6)*sin(q3)*sin(q4)+cos(q3)*sin(q5)*sin(q6)+cos(q4)*cos(q5)*sin(q3)*sin(q6))*7.921875E-5;
  C_row_7[1][0] = qd1*(cos(q4)*cos(q6)*sin(q2)*1.584375E-4+cos(q2)*cos(q3)*cos(q6)*sin(q4)*1.584375E-4-cos(q2)*sin(q3)*sin(q5)*sin(q6)*1.584375E-4-cos(q5)*sin(q2)*sin(q4)*sin(q6)*1.584375E-4+cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*1.584375E-4)*(-1.0/2.0)+qd3*(-sin(q3)*sin(q5)*sin(q6)+cos(q3)*cos(q6)*sin(q4)+cos(q3)*cos(q4)*cos(q5)*sin(q6))*7.921875E-5+qd6*(-sin(q3)*sin(q4)*sin(q6)+cos(q3)*cos(q6)*sin(q5)+cos(q4)*cos(q5)*cos(q6)*sin(q3))*7.921875E-5+qd5*sin(q6)*(cos(q3)*cos(q5)-cos(q4)*sin(q3)*sin(q5))*7.921875E-5+qd4*sin(q3)*(cos(q4)*cos(q6)-cos(q5)*sin(q4)*sin(q6))*7.921875E-5;
  C_row_7[2][0] = qd4*(cos(q6)*sin(q4)+cos(q4)*cos(q5)*sin(q6))*(-7.921875E-5)-qd6*(cos(q4)*sin(q6)+cos(q5)*cos(q6)*sin(q4))*7.921875E-5+qd2*(-sin(q3)*sin(q5)*sin(q6)+cos(q3)*cos(q6)*sin(q4)+cos(q3)*cos(q4)*cos(q5)*sin(q6))*7.921875E-5+qd1*sin(q2)*(cos(q6)*sin(q3)*sin(q4)+cos(q3)*sin(q5)*sin(q6)+cos(q4)*cos(q5)*sin(q3)*sin(q6))*7.921875E-5+qd5*sin(q4)*sin(q5)*sin(q6)*7.921875E-5;
  C_row_7[3][0] = qd1*(cos(q2)*cos(q6)*sin(q4)*1.584375E-4+cos(q3)*cos(q4)*cos(q6)*sin(q2)*1.584375E-4+cos(q2)*cos(q4)*cos(q5)*sin(q6)*1.584375E-4-cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6)*1.584375E-4)*(-1.0/2.0)-qd3*(cos(q6)*sin(q4)*1.584375E-4+cos(q4)*cos(q5)*sin(q6)*1.584375E-4)*(1.0/2.0)+qd2*sin(q3)*(cos(q4)*cos(q6)-cos(q5)*sin(q4)*sin(q6))*7.921875E-5+qd5*cos(q5)*sin(q6)*7.921875E-5+qd6*cos(q6)*sin(q5)*7.921875E-5;
  C_row_7[4][0] = sin(q6)*(-qd6+qd4*cos(q5)+qd2*cos(q3)*cos(q5)+qd3*sin(q4)*sin(q5)+qd1*cos(q5)*sin(q2)*sin(q3)+qd1*cos(q2)*sin(q4)*sin(q5)-qd2*cos(q4)*sin(q3)*sin(q5)+qd1*cos(q3)*cos(q4)*sin(q2)*sin(q5))*7.921875E-5;
  C_row_7[5][0] = qd5*sin(q6)*(-7.921875E-5)-qd3*cos(q4)*sin(q6)*7.921875E-5+qd4*cos(q6)*sin(q5)*7.921875E-5-qd1*cos(q2)*cos(q4)*sin(q6)*7.921875E-5+qd2*cos(q3)*cos(q6)*sin(q5)*7.921875E-5-qd3*cos(q5)*cos(q6)*sin(q4)*7.921875E-5-qd2*sin(q3)*sin(q4)*sin(q6)*7.921875E-5-qd1*cos(q2)*cos(q5)*cos(q6)*sin(q4)*7.921875E-5+qd2*cos(q4)*cos(q5)*cos(q6)*sin(q3)*7.921875E-5+qd1*cos(q3)*sin(q2)*sin(q4)*sin(q6)*7.921875E-5+qd1*cos(q6)*sin(q2)*sin(q3)*sin(q5)*7.921875E-5-qd1*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*7.921875E-5;
}
 

