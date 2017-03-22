#include "LBR4pSimple_gravload.h"

void LBR4pSimple_gravload(double G[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
 
        G[0][0]=0;
 G[1][0]=0;
 G[2][0]=0;
 G[3][0]=0;
 G[4][0]=0;
 G[5][0]=0;
 G[6][0]=0;



  G[1][0] = sin(q2)*(-3.5316E1)-cos(q4)*sin(q2)*1.377324E1-cos(q2)*cos(q3)*sin(q4)*1.377324E1-cos(q4)*cos(q6)*sin(q2)*4.467474E-1-cos(q2)*cos(q3)*cos(q6)*sin(q4)*4.467474E-1+cos(q2)*sin(q3)*sin(q5)*sin(q6)*4.467474E-1+cos(q5)*sin(q2)*sin(q4)*sin(q6)*4.467474E-1-cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*4.467474E-1;
  G[2][0] = sin(q2)*(sin(q3)*sin(q4)*7.8E3+cos(q6)*sin(q3)*sin(q4)*2.53E2+cos(q3)*sin(q5)*sin(q6)*2.53E2+cos(q4)*cos(q5)*sin(q3)*sin(q6)*2.53E2)*1.7658E-3;
  G[3][0] = cos(q2)*sin(q4)*(-1.377324E1)-cos(q3)*cos(q4)*sin(q2)*1.377324E1-cos(q2)*cos(q6)*sin(q4)*4.467474E-1-cos(q3)*cos(q4)*cos(q6)*sin(q2)*4.467474E-1-cos(q2)*cos(q4)*cos(q5)*sin(q6)*4.467474E-1+cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6)*4.467474E-1;
  G[4][0] = sin(q6)*(cos(q5)*sin(q2)*sin(q3)+cos(q2)*sin(q4)*sin(q5)+cos(q3)*cos(q4)*sin(q2)*sin(q5))*4.467474E-1;
  G[5][0] = cos(q2)*cos(q4)*sin(q6)*(-4.467474E-1)-cos(q2)*cos(q5)*cos(q6)*sin(q4)*4.467474E-1+cos(q3)*sin(q2)*sin(q4)*sin(q6)*4.467474E-1+cos(q6)*sin(q2)*sin(q3)*sin(q5)*4.467474E-1-cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*4.467474E-1;
}
 

