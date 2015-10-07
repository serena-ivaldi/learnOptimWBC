#include "Puma560_coriolis.h"

void Puma560_coriolis(double C[][6], const double* input1, const double* input2){

	/* allocate memory for individual rows */
	double row1[6][1];
	double row2[6][1];
	double row3[6][1];
	double row4[6][1];
	double row5[6][1];
	double row6[6][1];
 
	/* call the row routines */
	Puma560_coriolis_row_1(row1, input1, input2);
	Puma560_coriolis_row_2(row2, input1, input2);
	Puma560_coriolis_row_3(row3, input1, input2);
	Puma560_coriolis_row_4(row4, input1, input2);
	Puma560_coriolis_row_5(row5, input1, input2);
	Puma560_coriolis_row_6(row6, input1, input2);
 
	C[0][0] = row1[0][0];
	C[1][0] = row1[1][0];
	C[2][0] = row1[2][0];
	C[3][0] = row1[3][0];
	C[4][0] = row1[4][0];
	C[5][0] = row1[5][0];
 
	C[0][1] = row2[0][0];
	C[1][1] = row2[1][0];
	C[2][1] = row2[2][0];
	C[3][1] = row2[3][0];
	C[4][1] = row2[4][0];
	C[5][1] = row2[5][0];
 
	C[0][2] = row3[0][0];
	C[1][2] = row3[1][0];
	C[2][2] = row3[2][0];
	C[3][2] = row3[3][0];
	C[4][2] = row3[4][0];
	C[5][2] = row3[5][0];
 
	C[0][3] = row4[0][0];
	C[1][3] = row4[1][0];
	C[2][3] = row4[2][0];
	C[3][3] = row4[3][0];
	C[4][3] = row4[4][0];
	C[5][3] = row4[5][0];
 
	C[0][4] = row5[0][0];
	C[1][4] = row5[1][0];
	C[2][4] = row5[2][0];
	C[3][4] = row5[3][0];
	C[4][4] = row5[4][0];
	C[5][4] = row5[5][0];
 
	C[0][5] = row6[0][0];
	C[1][5] = row6[1][0];
	C[2][5] = row6[2][0];
	C[3][5] = row6[3][0];
	C[4][5] = row6[4][0];
	C[5][5] = row6[5][0];
 
}
