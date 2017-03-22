#include "LBR4pSimple_accel.h"

void LBR4pSimple_accel(double QDD[][7], const double* input1, const double* input2, const double* input3){

	/* declare variables */
	int iCol;
	double inertia[7][7];
	double invinertia[7][7];
	double coriolis[7][7];
	double gravload[7][1];
	double friction[7][1];
	double tmpTau[1][7];
 
	/* call the computational routines */
	LBR4pSimple_inertia(inertia, input1);
	gaussjordan(inertia, invinertia, 7);
	LBR4pSimple_coriolis(coriolis, input1, input2);
	LBR4pSimple_gravload(gravload, input1);
	LBR4pSimple_friction(friction, input2);
 
	/* fill temporary vector */
	matvecprod(tmpTau, coriolis, input2,7,7);
	for (iCol = 0; iCol < 7; iCol++){
		tmpTau[0][iCol] = input3[iCol] -  tmpTau[0][iCol] - gravload[iCol][0] + friction[iCol][0];
	}
	/* compute acceleration */
	matvecprod(QDD, invinertia, tmpTau,7,7);
}
