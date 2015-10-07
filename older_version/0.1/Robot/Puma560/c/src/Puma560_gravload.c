#include "Puma560_gravload.h"

void Puma560_gravload(double G[][1], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
 
       G[0][0]=0;
 G[1][0]=0;
 G[2][0]=0;
 G[3][0]=0;
 G[4][0]=0;
 G[5][0]=0;



  G[1][0] = cos(q2)*3.72347379E1-sin(q2)*1.024164+cos(q2)*cos(q3)*2.4892875E-1-cos(q2)*sin(q3)*8.7439473-cos(q3)*sin(q2)*8.7439473-sin(q2)*sin(q3)*2.4892875E-1-cos(q2)*cos(q5)*sin(q3)*2.82528E-2-cos(q3)*cos(q5)*sin(q2)*2.82528E-2-cos(q2)*cos(q3)*cos(q4)*sin(q5)*2.82528E-2+cos(q4)*sin(q2)*sin(q3)*sin(q5)*2.82528E-2;
  G[2][0] = cos(q2)*cos(q3)*2.4892875E-1-cos(q2)*sin(q3)*8.7439473-cos(q3)*sin(q2)*8.7439473-sin(q2)*sin(q3)*2.4892875E-1-cos(q2)*cos(q5)*sin(q3)*2.82528E-2-cos(q3)*cos(q5)*sin(q2)*2.82528E-2-cos(q2)*cos(q3)*cos(q4)*sin(q5)*2.82528E-2+cos(q4)*sin(q2)*sin(q3)*sin(q5)*2.82528E-2;
  G[3][0] = sin(q2+q3)*sin(q4)*sin(q5)*2.82528E-2;
  G[4][0] = sin(q2)*sin(q3)*sin(q5)*2.82528E-2-cos(q2)*cos(q3)*sin(q5)*2.82528E-2-cos(q2)*cos(q4)*cos(q5)*sin(q3)*2.82528E-2-cos(q3)*cos(q4)*cos(q5)*sin(q2)*2.82528E-2;
}
 

