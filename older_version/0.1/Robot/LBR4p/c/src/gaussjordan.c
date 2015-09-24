#include "gaussjordan.h"

void gaussjordan(const double* inMatrix, double* outMatrix, int dim){
 
	int iRow, iCol, diagIndex;
	double diagFactor, tmpFactor;
	double* inMatrixCopy = (double*) malloc(dim*dim*sizeof(double));
 
	/* make deep copy of input matrix */
	for(iRow = 0; iRow < dim; iRow++ ){
		for (iCol = 0; iCol < dim; iCol++){
			inMatrixCopy[dim*iCol+iRow] = inMatrix[dim*iCol+iRow];
		}
	}
	/* Initialize output matrix as identity matrix. */
	for (iRow = 0; iRow < dim; iRow++ ){
		for (iCol = 0; iCol < dim; iCol++ ){
			if (iCol == iRow){
				outMatrix[dim*iCol+iRow] = 1;
			}
			else{
				outMatrix[dim*iCol+iRow] = 0;
			}
		}
	}
 
	for (diagIndex = 0; diagIndex < dim; diagIndex++ )
	{
		/* determine diagonal factor */
		diagFactor = inMatrixCopy[dim*diagIndex+diagIndex];
 
		/* divide column entries by diagonal factor */
		for (iCol = 0; iCol < dim; iCol++){
			inMatrixCopy[dim*iCol+diagIndex] /= diagFactor;
			outMatrix[dim*iCol+diagIndex] /= diagFactor;
		}
 
		/* perform line-by-line elimination */
		for (iRow = 0; iRow < dim; iRow++){
			if (iRow != diagIndex){
				tmpFactor = inMatrixCopy[dim*diagIndex+iRow];
 
				for(iCol = 0; iCol < dim; iCol++){
				inMatrixCopy[dim*iCol+iRow]  -= inMatrixCopy[dim*iCol+diagIndex]*tmpFactor;
				outMatrix[dim*iCol+iRow] -= outMatrix[dim*iCol+diagIndex]*tmpFactor;
				}
			}
		} /* line-by-line elimination */
 
	}
	free(inMatrixCopy);
}
