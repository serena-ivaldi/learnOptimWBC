#include "Puma560_accel.h"

void Puma560_accel(double QDD[][6], const double* input1, const double* input2, const double* input3){

	/* declare variables */
	int iCol;
	double inertia[6][6];
	double invinertia[6][6];
	double coriolis[6][6];
	double gravload[6][1];
	double friction[6][1];
	double tmpTau[1][6];
 
	/* call the computational routines */
	Puma560_inertia(inertia, input1);
	gaussjordan(inertia, invinertia, 6);
	Puma560_coriolis(coriolis, input1, input2);
	Puma560_gravload(gravload, input1);
	Puma560_friction(friction, input2);
 
	/* fill temporary vector */
	matvecprod(tmpTau, coriolis, input2,6,6);
	for (iCol = 0; iCol < 6; iCol++){
		tmpTau[0][iCol] = input3[iCol] -  tmpTau[0][iCol] - gravload[iCol][0] + friction[iCol][0];
	}
	/* compute acceleration */
	matvecprod(QDD, invinertia, tmpTau,6,6);
}
