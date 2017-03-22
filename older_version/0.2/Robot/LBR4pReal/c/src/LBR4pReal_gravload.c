#include "LBR4pReal_gravload.h"

void LBR4pReal_gravload(double G[][1], const double* input1){
 
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



  G[1][0] = cos(q2)*(-3.549258E-2)-sin(q2)*3.5316E1+cos(q2)*cos(q3)*3.549258E-2-cos(q4)*sin(q2)*1.7960176449E1-sin(q2)*sin(q4)*3.549258E-2+sin(q2)*sin(q4)*sin(q5)*5.43063942E-1+cos(q2)*cos(q3)*cos(q4)*3.549258E-2-cos(q2)*cos(q3)*sin(q4)*1.7960176449E1-cos(q2)*cos(q5)*sin(q3)*5.43063942E-1-cos(q4)*cos(q6)*sin(q2)*3.31334712E-1-cos(q2)*sin(q3)*sin(q5)*1.6560261E-2-cos(q5)*sin(q2)*sin(q4)*1.6560261E-2-cos(q4)*sin(q2)*sin(q6)*4.065264E-3+cos(q2)*cos(q3)*cos(q4)*cos(q5)*1.6560261E-2-cos(q2)*cos(q3)*cos(q4)*sin(q5)*5.43063942E-1-cos(q2)*cos(q3)*cos(q6)*sin(q4)*3.31334712E-1-cos(q2)*cos(q3)*sin(q4)*sin(q6)*4.065264E-3-cos(q2)*cos(q6)*sin(q3)*sin(q5)*4.065264E-3-cos(q5)*cos(q6)*sin(q2)*sin(q4)*4.065264E-3+cos(q2)*sin(q3)*sin(q5)*sin(q6)*3.31334712E-1+cos(q5)*sin(q2)*sin(q4)*sin(q6)*3.31334712E-1+cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*4.065264E-3-cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*3.31334712E-1;
  G[2][0] = sin(q2)*(sin(q3)*6.517609380730582E20+cos(q3)*cos(q5)*9.972446755676056E21+cos(q4)*sin(q3)*6.517609380730582E20+cos(q3)*sin(q5)*3.041010612385654E20-sin(q3)*sin(q4)*3.298081303291529E23-sin(q3)*sin(q4)*sin(q6)*7.465166742329334E19+cos(q4)*cos(q5)*sin(q3)*3.041010612385654E20+cos(q3)*cos(q6)*sin(q5)*7.465166742329334E19-cos(q4)*sin(q3)*sin(q5)*9.972446755676056E21-cos(q6)*sin(q3)*sin(q4)*6.084399125374559E21-cos(q3)*sin(q5)*sin(q6)*6.084399125374559E21+cos(q4)*cos(q5)*cos(q6)*sin(q3)*7.465166742329334E19-cos(q4)*cos(q5)*sin(q3)*sin(q6)*6.084399125374559E21)*(-5.445643935786393E-23);
  G[3][0] = cos(q2)*cos(q4)*3.549258E-2-cos(q2)*sin(q4)*1.7960176449E1+cos(q2)*cos(q4)*cos(q5)*1.6560261E-2-cos(q3)*cos(q4)*sin(q2)*1.7960176449E1-cos(q2)*cos(q4)*sin(q5)*5.43063942E-1-cos(q2)*cos(q6)*sin(q4)*3.31334712E-1-cos(q3)*sin(q2)*sin(q4)*3.549258E-2-cos(q2)*sin(q4)*sin(q6)*4.065264E-3+cos(q2)*cos(q4)*cos(q5)*cos(q6)*4.065264E-3-cos(q3)*cos(q4)*cos(q6)*sin(q2)*3.31334712E-1-cos(q2)*cos(q4)*cos(q5)*sin(q6)*3.31334712E-1-cos(q3)*cos(q5)*sin(q2)*sin(q4)*1.6560261E-2-cos(q3)*cos(q4)*sin(q2)*sin(q6)*4.065264E-3+cos(q3)*sin(q2)*sin(q4)*sin(q5)*5.43063942E-1-cos(q3)*cos(q5)*cos(q6)*sin(q2)*sin(q4)*4.065264E-3+cos(q3)*cos(q5)*sin(q2)*sin(q4)*sin(q6)*3.31334712E-1;
  G[4][0] = sin(q2)*sin(q3)*sin(q5)*5.43063942E-1-cos(q2)*cos(q5)*sin(q4)*5.43063942E-1-cos(q5)*sin(q2)*sin(q3)*1.6560261E-2-cos(q2)*sin(q4)*sin(q5)*1.6560261E-2-cos(q3)*cos(q4)*cos(q5)*sin(q2)*5.43063942E-1-cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.6560261E-2-cos(q5)*cos(q6)*sin(q2)*sin(q3)*4.065264E-3-cos(q2)*cos(q6)*sin(q4)*sin(q5)*4.065264E-3+cos(q5)*sin(q2)*sin(q3)*sin(q6)*3.31334712E-1+cos(q2)*sin(q4)*sin(q5)*sin(q6)*3.31334712E-1-cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5)*4.065264E-3+cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)*3.31334712E-1;
  G[5][0] = cos(q2)*cos(q4)*cos(q6)*4.065264E-3-cos(q2)*cos(q4)*sin(q6)*3.31334712E-1-cos(q2)*cos(q5)*cos(q6)*sin(q4)*3.31334712E-1-cos(q3)*cos(q6)*sin(q2)*sin(q4)*4.065264E-3-cos(q2)*cos(q5)*sin(q4)*sin(q6)*4.065264E-3+cos(q3)*sin(q2)*sin(q4)*sin(q6)*3.31334712E-1+cos(q6)*sin(q2)*sin(q3)*sin(q5)*3.31334712E-1+sin(q2)*sin(q3)*sin(q5)*sin(q6)*4.065264E-3-cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*3.31334712E-1-cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*4.065264E-3;
}
 

