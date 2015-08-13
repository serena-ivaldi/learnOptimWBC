#include "Jaco_fkine.h"

void Jaco_fkine(double T[][4], const double* input1){
 
  double q1 = input1[0];
  double q2 = input1[1];
  double q3 = input1[2];
  double q4 = input1[3];
  double q5 = input1[4];
  double q6 = input1[5];
 
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



  T[0][0] = -sin(3.141592653589793*(1.0/1.8E1)+q6)*(cos(q5)*(-sin(q1)*sin(q4)+cos(q1)*cos(q2)*cos(q3)*cos(q4)+cos(q1)*cos(q4)*sin(q2)*sin(q3))-sin(q5)*(cos(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)))-cos(3.141592653589793*(1.0/1.8E1)+q6)*(sin(3.141592653589793*(1.1E1/3.6E1))*(sin(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4))+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q5)*(-sin(q1)*sin(q4)+cos(q1)*cos(q2)*cos(q3)*cos(q4)+cos(q1)*cos(q4)*sin(q2)*sin(q3))+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(cos(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)));
  T[0][1] = cos(3.141592653589793*(1.0/1.8E1)+q6)*(sin(3.141592653589793*(1.1E1/3.6E1))*(-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4))+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q5)*(cos(q1)*sin(q4)+cos(q2)*cos(q3)*cos(q4)*sin(q1)+cos(q4)*sin(q1)*sin(q2)*sin(q3))+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4)))+sin(3.141592653589793*(1.0/1.8E1)+q6)*(cos(q5)*(cos(q1)*sin(q4)+cos(q2)*cos(q3)*cos(q4)*sin(q1)+cos(q4)*sin(q1)*sin(q2)*sin(q3))-sin(q5)*(-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4)));
  T[0][2] = -sin(3.141592653589793*(1.0/1.8E1)+q6)*(sin(q5)*(sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4))+sin(q2-q3)*cos(q4)*cos(q5))-cos(3.141592653589793*(1.0/1.8E1)+q6)*(sin(3.141592653589793*(1.1E1/3.6E1))*(cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4))-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4))+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q2-q3)*cos(q4)*sin(q5));
  T[1][0] = cos(3.141592653589793*(1.0/1.8E1)+q6)*(cos(q5)*(-sin(q1)*sin(q4)+cos(q1)*cos(q2)*cos(q3)*cos(q4)+cos(q1)*cos(q4)*sin(q2)*sin(q3))-sin(q5)*(cos(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)))-sin(3.141592653589793*(1.0/1.8E1)+q6)*(sin(3.141592653589793*(1.1E1/3.6E1))*(sin(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4))+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q5)*(-sin(q1)*sin(q4)+cos(q1)*cos(q2)*cos(q3)*cos(q4)+cos(q1)*cos(q4)*sin(q2)*sin(q3))+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(cos(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)));
  T[1][1] = sin(3.141592653589793*(1.0/1.8E1)+q6)*(sin(3.141592653589793*(1.1E1/3.6E1))*(-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4))+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q5)*(cos(q1)*sin(q4)+cos(q2)*cos(q3)*cos(q4)*sin(q1)+cos(q4)*sin(q1)*sin(q2)*sin(q3))+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4)))-cos(3.141592653589793*(1.0/1.8E1)+q6)*(cos(q5)*(cos(q1)*sin(q4)+cos(q2)*cos(q3)*cos(q4)*sin(q1)+cos(q4)*sin(q1)*sin(q2)*sin(q3))-sin(q5)*(-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4)));
  T[1][2] = -sin(3.141592653589793*(1.0/1.8E1)+q6)*(sin(3.141592653589793*(1.1E1/3.6E1))*(cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4))-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4))+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q2-q3)*cos(q4)*sin(q5))+cos(3.141592653589793*(1.0/1.8E1)+q6)*(sin(q5)*(sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4))+sin(q2-q3)*cos(q4)*cos(q5));
  T[2][0] = cos(3.141592653589793*(1.1E1/3.6E1))*(sin(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4))-sin(3.141592653589793*(1.1E1/3.6E1))*sin(q5)*(-sin(q1)*sin(q4)+cos(q1)*cos(q2)*cos(q3)*cos(q4)+cos(q1)*cos(q4)*sin(q2)*sin(q3))-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(cos(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4));
  T[2][1] = -cos(3.141592653589793*(1.1E1/3.6E1))*(-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4))+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q5)*(cos(q1)*sin(q4)+cos(q2)*cos(q3)*cos(q4)*sin(q1)+cos(q4)*sin(q1)*sin(q2)*sin(q3))+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4));
  T[2][2] = cos(3.141592653589793*(1.1E1/3.6E1))*(cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4))+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*(sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4))-sin(q2-q3)*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q5);
  T[3][0] = sin(q1)*(-9.8E-3)+cos(q1)*sin(q2)*(4.1E1/1.0E2)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)*8.376447865216925E-2+cos(q1)*cos(q2)*sin(q3)*2.491822393260846E-1-cos(q1)*cos(q3)*sin(q2)*2.491822393260846E-1+sin(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q4)*sin(q5)*2.105822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)*2.105822393260846E-1+pow(cos(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q1)*cos(q2)*sin(q3)*2.105822393260846E-1-pow(cos(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q1)*cos(q3)*sin(q2)*2.105822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*sin(q3)*8.376447865216925E-2-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q3)*sin(q2)*8.376447865216925E-2+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)*8.376447865216925E-2-cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*cos(q5)*sin(q1)*2.105822393260846E-1+pow(sin(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q1)*cos(q2)*cos(q5)*sin(q3)*2.105822393260846E-1-pow(sin(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q1)*cos(q3)*cos(q5)*sin(q2)*2.105822393260846E-1+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4)*8.376447865216925E-2+cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q2)*sin(q3)*sin(q4)*2.105822393260846E-1-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*2.105822393260846E-1-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*2.105822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*sin(q4)*2.105822393260846E-1-cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q2)*cos(q3)*cos(q5)*sin(q4)*2.105822393260846E-1-cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q5)*sin(q2)*sin(q3)*sin(q4)*2.105822393260846E-1;
  T[3][1] = cos(q1)*(-9.8E-3)-sin(q1)*sin(q2)*(4.1E1/1.0E2)+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)*8.376447865216925E-2-cos(q2)*sin(q1)*sin(q3)*2.491822393260846E-1+cos(q3)*sin(q1)*sin(q2)*2.491822393260846E-1-cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q1)*sin(q3)*8.376447865216925E-2+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q1)*sin(q2)*8.376447865216925E-2+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*sin(q4)*sin(q5)*2.105822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)*2.105822393260846E-1-pow(cos(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q2)*sin(q1)*sin(q3)*2.105822393260846E-1+pow(cos(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q3)*sin(q1)*sin(q2)*2.105822393260846E-1-cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q1)*cos(q4)*cos(q5)*2.105822393260846E-1-sin(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)*8.376447865216925E-2-pow(sin(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q2)*cos(q5)*sin(q1)*sin(q3)*2.105822393260846E-1+pow(sin(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q3)*cos(q5)*sin(q1)*sin(q2)*2.105822393260846E-1-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4)*8.376447865216925E-2-cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*sin(q1)*sin(q4)*2.105822393260846E-1-cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*sin(q1)*sin(q2)*sin(q3)*sin(q4)*2.105822393260846E-1+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*2.105822393260846E-1+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*2.105822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4)*2.105822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*2.105822393260846E-1;
  T[3][2] = cos(q2)*(-4.1E1/1.0E2)+cos(q2)*cos(q3)*2.491822393260846E-1+sin(q2)*sin(q3)*2.491822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q3)*8.376447865216925E-2+cos(3.141592653589793*(1.1E1/3.6E1))*sin(q2)*sin(q3)*8.376447865216925E-2+pow(cos(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q2)*cos(q3)*2.105822393260846E-1+pow(cos(3.141592653589793*(1.1E1/3.6E1)),2.0)*sin(q2)*sin(q3)*2.105822393260846E-1-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)*8.376447865216925E-2+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4)*8.376447865216925E-2+pow(sin(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q2)*cos(q3)*cos(q5)*2.105822393260846E-1+pow(sin(3.141592653589793*(1.1E1/3.6E1)),2.0)*cos(q5)*sin(q2)*sin(q3)*2.105822393260846E-1-cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*sin(q3)*sin(q4)*2.105822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*sin(q2)*sin(q4)*2.105822393260846E-1+sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q4)*sin(q3)*sin(q5)*2.105822393260846E-1-sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*cos(q4)*sin(q2)*sin(q5)*2.105822393260846E-1+cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q2)*cos(q5)*sin(q3)*sin(q4)*2.105822393260846E-1-cos(3.141592653589793*(1.1E1/3.6E1))*sin(3.141592653589793*(1.1E1/3.6E1))*cos(q3)*cos(q5)*sin(q2)*sin(q4)*2.105822393260846E-1+2.755E-1;
  T[3][3] = 1.0;
}
 

