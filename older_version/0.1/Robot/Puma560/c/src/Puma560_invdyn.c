#include "Puma560_invdyn.h"

void Puma560_invdyn(double TAU[][6], const double* input1, const double* input2, const double* input3){

/* declare variables */
double inertia_row1[6][1];
double inertia_row2[6][1];
double inertia_row3[6][1];
double inertia_row4[6][1];
double inertia_row5[6][1];
double inertia_row6[6][1];
double coriolis_row1[6][1];
double coriolis_row2[6][1];
double coriolis_row3[6][1];
double coriolis_row4[6][1];
double coriolis_row5[6][1];
double coriolis_row6[6][1];
double gravload[6][1];
double friction[6][1];
 
/* call the computational routines */
Puma560_gravload(gravload, input1);
Puma560_friction(friction, input2);
/* rowwise routines */
Puma560_inertia_row_1(inertia_row1, input1);
Puma560_inertia_row_2(inertia_row2, input1);
Puma560_inertia_row_3(inertia_row3, input1);
Puma560_inertia_row_4(inertia_row4, input1);
Puma560_inertia_row_5(inertia_row5, input1);
Puma560_inertia_row_6(inertia_row6, input1);
Puma560_coriolis_row_1(coriolis_row1, input1, input2);
Puma560_coriolis_row_2(coriolis_row2, input1, input2);
Puma560_coriolis_row_3(coriolis_row3, input1, input2);
Puma560_coriolis_row_4(coriolis_row4, input1, input2);
Puma560_coriolis_row_5(coriolis_row5, input1, input2);
Puma560_coriolis_row_6(coriolis_row6, input1, input2);
 
/* fill output vector */
TAU[0][0] = dotprod(inertia_row1, input3, 6) /* inertia */
	 + dotprod(coriolis_row1, input2, 6) /* coriolis */
	 + gravload[0][0]
	 - friction[0][0];
TAU[0][1] = dotprod(inertia_row2, input3, 6) /* inertia */
	 + dotprod(coriolis_row2, input2, 6) /* coriolis */
	 + gravload[1][0]
	 - friction[1][0];
TAU[0][2] = dotprod(inertia_row3, input3, 6) /* inertia */
	 + dotprod(coriolis_row3, input2, 6) /* coriolis */
	 + gravload[2][0]
	 - friction[2][0];
TAU[0][3] = dotprod(inertia_row4, input3, 6) /* inertia */
	 + dotprod(coriolis_row4, input2, 6) /* coriolis */
	 + gravload[3][0]
	 - friction[3][0];
TAU[0][4] = dotprod(inertia_row5, input3, 6) /* inertia */
	 + dotprod(coriolis_row5, input2, 6) /* coriolis */
	 + gravload[4][0]
	 - friction[4][0];
TAU[0][5] = dotprod(inertia_row6, input3, 6) /* inertia */
	 + dotprod(coriolis_row6, input2, 6) /* coriolis */
	 + gravload[5][0]
	 - friction[5][0];
 
}
