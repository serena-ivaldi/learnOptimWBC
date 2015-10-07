#include "LBR4p_friction.h"

void LBR4p_friction(double F[][1], const double* input1){
 
  double qd1 = input1[0];
  double qd2 = input1[1];
  double qd3 = input1[2];
  double qd4 = input1[3];
  double qd5 = input1[4];
  double qd6 = input1[5];
  double qd7 = input1[6];
 
        F[0][0]=0;
 F[1][0]=0;
 F[2][0]=0;
 F[3][0]=0;
 F[4][0]=0;
 F[5][0]=0;
 F[6][0]=0;



  F[0][0] = qd1*(-5.8018217679508)-2.47313845E1;
  F[1][0] = qd2*(-9.496868641825)-1.358469E1;
  F[2][0] = qd3*(-3.9804259903722)-7.0892316;
  F[3][0] = qd4*(-4.11645229697152E-1)-8.5160768E-1;
  F[4][0] = qd5*(-4.272830209354E-1)-6.6600698E-1;
  F[5][0] = qd6*(-2.158232532732E-1)-3.0367656E-1;
  F[6][0] = qd7*(-2.158232532732E-1)-3.0367656E-1;
}
 

