/* LBR4PREAL_T0_1 - This file contains auto generated C-code for a MATLAB MEX function.
// For details on how to use the complied MEX function see the documentation provided in LBR4pReal_T0_1.m
// The compiled MEX function replaces this .m-function with identical usage but substantial execution speedup.
//
// For compilation of this C-code using MATLAB please run:
//
// 		'mex /home/vale/git/learnOptimWBC/matlab/Robot/LBR4pReal/@LBR4pReal/T0_1.c /home/vale/git/learnOptimWBC/matlab/Robot/LBR4pReal/c/src/LBR4pReal_T0_1.c -I/home/vale/git/learnOptimWBC/matlab/Robot/LBR4pReal/c/include -v -outdir /home/vale/git/learnOptimWBC/matlab/Robot/LBR4pReal/@LBR4pReal'
//
// Make sure you have a C-compiler installed and your MATLAB MEX environment readily configured.
// Type 'doc mex' for additional help.
//
// __Copyright Note__:
// Copyright (C) 1993-2015, by Peter I. Corke 
// Copyright (C) 2012-2015, by Joern Malzahn 
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
#include "LBR4pReal_T0_1.h"


/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray *prhs[])
{
	/* variable declarations */
	double* outMatrix;      /* output matrix */
	double* input1;
 
	/* check for proper number of arguments */
	if(nrhs!=2) {
		mexErrMsgIdAndTxt("LBR4pReal_T0_1:nrhs",
		                       "1 inputs required.");
	}
	if(nlhs>1) {
		mexErrMsgIdAndTxt("LBR4pReal_T0_1:nlhs",
		                       "Only single output allowed.");
	}
 
	/* allocate memory for the output matrix */
	plhs[0] = mxCreateDoubleMatrix(4,4, mxREAL);
	/* get a pointer to the real data in the output matrix */
	outMatrix = mxGetPr(plhs[0]);
 
	/* get a pointers to the real data in the input matrices */
	input1 = mxGetPr(prhs[1]);
 
	/* call the computational routine */
	LBR4pReal_T0_1(outMatrix, input1);
}
