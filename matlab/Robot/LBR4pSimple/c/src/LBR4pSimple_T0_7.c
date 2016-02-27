#include "LBR4pSimple_T0_7.h"

void LBR4pSimple_T0_7(double T[][4], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
  double q7 = input1[6];
 
                 T[0][0]=0;
 T[1][0]=0;
 T[2][0]=0;
 T[3][0]=0;
 T[0][1]=0;
 T[1][1]=0;
 T[2][1]=0;
 T[3][1]=0;
 T[0][2]=0;
 T[1][2]=0;
 T[2][2]=0;
 T[3][2]=0;
 T[0][3]=0;
 T[1][3]=0;
 T[2][3]=0;
 T[3][3]=0;



  T[0][0] = cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))-cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3))))+sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)));
  T[0][1] = -cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))-cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3))))-sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)));
  T[0][2] = sin(q7)*(sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3))-cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4)));
  T[1][0] = -sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))-cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3))))+cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)));
  T[1][1] = sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))-cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3))))-cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))-cos(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)));
  T[1][2] = cos(q7)*(sin(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))+cos(q5)*sin(q2)*sin(q3))+sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+sin(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4)));
  T[2][0] = -cos(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))-sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)));
  T[2][1] = cos(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))+sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)));
  T[2][2] = -sin(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))+cos(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4));
  T[3][0] = cos(q1)*sin(q2)*(2.0/5.0)-cos(q6)*(sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))-cos(q1)*cos(q4)*sin(q2))*(3.9E1/5.0E2)-sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))+cos(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q3)*sin(q1)+cos(q1)*cos(q2)*sin(q3)))*(3.9E1/5.0E2)-sin(q4)*(sin(q1)*sin(q3)-cos(q1)*cos(q2)*cos(q3))*(3.9E1/1.0E2)+cos(q1)*cos(q4)*sin(q2)*(3.9E1/1.0E2);
  T[3][1] = sin(q1)*sin(q2)*(2.0/5.0)+cos(q6)*(sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))+cos(q4)*sin(q1)*sin(q2))*(3.9E1/5.0E2)+sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))-sin(q1)*sin(q2)*sin(q4))+sin(q5)*(cos(q1)*cos(q3)-cos(q2)*sin(q1)*sin(q3)))*(3.9E1/5.0E2)+sin(q4)*(cos(q1)*sin(q3)+cos(q2)*cos(q3)*sin(q1))*(3.9E1/1.0E2)+cos(q4)*sin(q1)*sin(q2)*(3.9E1/1.0E2);
  T[3][2] = cos(q2)*(2.0/5.0)+cos(q2)*cos(q4)*(3.9E1/1.0E2)-sin(q6)*(cos(q5)*(cos(q2)*sin(q4)+cos(q3)*cos(q4)*sin(q2))-sin(q2)*sin(q3)*sin(q5))*(3.9E1/5.0E2)+cos(q6)*(cos(q2)*cos(q4)-cos(q3)*sin(q2)*sin(q4))*(3.9E1/5.0E2)-cos(q3)*sin(q2)*sin(q4)*(3.9E1/1.0E2)+3.1E1/1.0E2;
  T[3][3] = 1.0;
}
 

