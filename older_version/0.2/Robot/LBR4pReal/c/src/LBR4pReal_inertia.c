#include "LBR4pReal_inertia.h"

void LBR4pReal_inertia(double I[][7], const double* input1){

	/* allocate memory for individual rows */
	double row1[7][1];
	double row2[7][1];
	double row3[7][1];
	double row4[7][1];
	double row5[7][1];
	double row6[7][1];
	double row7[7][1];
 
	/* call the row routines */
	LBR4pReal_inertia_row_1(row1, input1);
	LBR4pReal_inertia_row_2(row2, input1);
	LBR4pReal_inertia_row_3(row3, input1);
	LBR4pReal_inertia_row_4(row4, input1);
	LBR4pReal_inertia_row_5(row5, input1);
	LBR4pReal_inertia_row_6(row6, input1);
	LBR4pReal_inertia_row_7(row7, input1);
 
	I[0][0] = row1[0][0];
	I[1][0] = row1[1][0];
	I[2][0] = row1[2][0];
	I[3][0] = row1[3][0];
	I[4][0] = row1[4][0];
	I[5][0] = row1[5][0];
	I[6][0] = row1[6][0];
 
	I[0][1] = row2[0][0];
	I[1][1] = row2[1][0];
	I[2][1] = row2[2][0];
	I[3][1] = row2[3][0];
	I[4][1] = row2[4][0];
	I[5][1] = row2[5][0];
	I[6][1] = row2[6][0];
 
	I[0][2] = row3[0][0];
	I[1][2] = row3[1][0];
	I[2][2] = row3[2][0];
	I[3][2] = row3[3][0];
	I[4][2] = row3[4][0];
	I[5][2] = row3[5][0];
	I[6][2] = row3[6][0];
 
	I[0][3] = row4[0][0];
	I[1][3] = row4[1][0];
	I[2][3] = row4[2][0];
	I[3][3] = row4[3][0];
	I[4][3] = row4[4][0];
	I[5][3] = row4[5][0];
	I[6][3] = row4[6][0];
 
	I[0][4] = row5[0][0];
	I[1][4] = row5[1][0];
	I[2][4] = row5[2][0];
	I[3][4] = row5[3][0];
	I[4][4] = row5[4][0];
	I[5][4] = row5[5][0];
	I[6][4] = row5[6][0];
 
	I[0][5] = row6[0][0];
	I[1][5] = row6[1][0];
	I[2][5] = row6[2][0];
	I[3][5] = row6[3][0];
	I[4][5] = row6[4][0];
	I[5][5] = row6[5][0];
	I[6][5] = row6[6][0];
 
	I[0][6] = row7[0][0];
	I[1][6] = row7[1][0];
	I[2][6] = row7[2][0];
	I[3][6] = row7[3][0];
	I[4][6] = row7[4][0];
	I[5][6] = row7[5][0];
	I[6][6] = row7[6][0];
 
}
