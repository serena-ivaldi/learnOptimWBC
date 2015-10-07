#include "dotprod.h"

double dotprod(const double *input1, const double *input2, int nEl){
	double res = 0;
	int iEl = 0;

	for (iEl = 0; iEl < nEl; iEl++){
		res += input1[iEl] * input2[iEl];
	}

	return res;
}
