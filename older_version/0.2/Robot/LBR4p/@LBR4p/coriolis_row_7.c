/* LBR4P_CORIOLIS_ROW_7 - This file contains auto generated C-code for a MATLAB MEX function.
// For details on how to use the complied MEX function see the documentation provided in LBR4p_coriolis_row_7.m
// The compiled MEX function replaces this .m-function with identical usage but substantial execution speedup.
//
// For compilation of this C-code using MATLAB please run:
//
// 		'mex LBR4p/@LBR4p/coriolis_row_7.c LBR4p/c/src/LBR4p_coriolis_row_7.c -ILBR4p/c/include -v -outdir LBR4p/@LBR4p'
//
// Make sure you have a C-compiler installed and your MATLAB MEX environment readily configured.
// Type 'doc mex' for additional help.
//
// __Copyright Note__:
// Copyright (C) 1993-2014, by Peter I. Corke 
// Copyright (C) 2012-2014, by Joern Malzahn 
// 
// This file has been automatically generated with The Robotics Toolbox for Matlab (RTB). 
// 
// RTB and code generated with RTB is free software: you can redistribute it and/or modify 
// it under the terms of the GNU Lesser General Public License as published by 
// the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
//  
// RTB is distributed in the hope that it will be useful, 
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU Lesser General Public License for more details. 
//  
// You should have received a copy of the GNU Leser General Public License 
// along with RTB.  If not, see <http://www.gnu.org/licenses/>. 
// 
// http://www.petercorke.com 
// 
// The code generation module emerged during the work on a project funded by 
// the German Research Foundation (DFG, BE1569/7-1). The authors gratefully  
// acknowledge the financial support. 
 */
#include "mex.h"
#include "LBR4p_coriolis_row_7.h"


/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[])
{
	/* variable declarations */
	double* outMatrix;      /* output matrix */
	double* input1;
	double* input2;
 
	/* check for proper number of arguments */
	if(nrhs!=3) {
		mexErrMsgIdAndTxt("LBR4p_coriolis_row_7:nrhs",
		                       "2 inputs required.");
	}
	if(nlhs>1) {
		mexErrMsgIdAndTxt("LBR4p_coriolis_row_7:nlhs",
		                       "Only single output allowed.");
	}
 
	/* allocate memory for the output matrix */
	plhs[0] = mxCreateDoubleMatrix(1,7, mxREAL);
	/* get a pointer to the real data in the output matrix */
	outMatrix = mxGetPr(plhs[0]);
 
	/* get a pointers to the real data in the input matrices */
	input1 = mxGetPr(prhs[1]);
	input2 = mxGetPr(prhs[2]);
 
	/* call the computational routine */
	LBR4p_coriolis_row_7(outMatrix, input1, input2);
}
