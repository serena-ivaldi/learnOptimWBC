#include "Puma560_coriolis_row_6.h"

void Puma560_coriolis_row_6(double C_row_6[][1], const double* input1, const double* input2){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double qd1 = input2[0];
  double qd2 = input2[1];
  double qd3 = input2[2];
  double qd4 = input2[3];
  double qd5 = input2[4];
  double qd6 = input2[5];
 
       C_row_6[0][0]=0;
 C_row_6[1][0]=0;
 C_row_6[2][0]=0;
 C_row_6[3][0]=0;
 C_row_6[4][0]=0;
 C_row_6[5][0]=0;



  C_row_6[0][0] = qd2*(sin(q2+q3)*cos(q5)*4.0E-5+cos(q2+q3)*cos(q4)*sin(q5)*4.0E-5)*(-1.0/2.0)-qd3*(sin(q2+q3)*cos(q5)*4.0E-5+cos(q2+q3)*cos(q4)*sin(q5)*4.0E-5)*(1.0/2.0)-qd5*(cos(q2+q3)*sin(q5)*4.0E-5+sin(q2+q3)*cos(q4)*cos(q5)*4.0E-5)*(1.0/2.0)+qd4*sin(q2+q3)*sin(q4)*sin(q5)*2.0E-5;
  C_row_6[1][0] = qd1*(sin(q2+q3)*cos(q5)*4.0E-5+cos(q2+q3)*cos(q4)*sin(q5)*4.0E-5)*(-1.0/2.0)+qd4*cos(q4)*sin(q5)*2.0E-5+qd5*cos(q5)*sin(q4)*2.0E-5;
  C_row_6[2][0] = qd1*(sin(q2+q3)*cos(q5)*4.0E-5+cos(q2+q3)*cos(q4)*sin(q5)*4.0E-5)*(-1.0/2.0)+qd4*cos(q4)*sin(q5)*2.0E-5+qd5*cos(q5)*sin(q4)*2.0E-5;
  C_row_6[3][0] = sin(q5)*(-qd5+qd2*cos(q4)+qd3*cos(q4)+qd1*sin(q2+q3)*sin(q4))*2.0E-5;
  C_row_6[4][0] = qd4*sin(q5)*(-2.0E-5)-qd1*(cos(q2+q3)*sin(q5)*4.0E-5+sin(q2+q3)*cos(q4)*cos(q5)*4.0E-5)*(1.0/2.0)+qd2*cos(q5)*sin(q4)*2.0E-5+qd3*cos(q5)*sin(q4)*2.0E-5;
}
 

