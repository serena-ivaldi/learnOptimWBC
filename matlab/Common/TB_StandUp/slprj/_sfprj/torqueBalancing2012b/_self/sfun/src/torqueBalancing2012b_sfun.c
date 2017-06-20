/* Include files */

#include "torqueBalancing2012b_sfun.h"
#include "torqueBalancing2012b_sfun_debug_macros.h"
#include "c2_torqueBalancing2012b.h"
#include "c3_torqueBalancing2012b.h"
#include "c4_torqueBalancing2012b.h"
#include "c5_torqueBalancing2012b.h"
#include "c6_torqueBalancing2012b.h"
#include "c7_torqueBalancing2012b.h"
#include "c8_torqueBalancing2012b.h"
#include "c9_torqueBalancing2012b.h"
#include "c10_torqueBalancing2012b.h"
#include "c11_torqueBalancing2012b.h"
#include "c12_torqueBalancing2012b.h"
#include "c13_torqueBalancing2012b.h"
#include "c14_torqueBalancing2012b.h"
#include "c15_torqueBalancing2012b.h"
#include "c16_torqueBalancing2012b.h"
#include "c17_torqueBalancing2012b.h"
#include "c18_torqueBalancing2012b.h"
#include "c19_torqueBalancing2012b.h"
#include "c20_torqueBalancing2012b.h"
#include "c21_torqueBalancing2012b.h"
#include "c26_torqueBalancing2012b.h"
#include "c28_torqueBalancing2012b.h"
#include "c29_torqueBalancing2012b.h"
#include "c30_torqueBalancing2012b.h"
#include "c31_torqueBalancing2012b.h"
#include "c32_torqueBalancing2012b.h"
#include "c33_torqueBalancing2012b.h"
#include "c34_torqueBalancing2012b.h"
#include "c35_torqueBalancing2012b.h"
#include "c36_torqueBalancing2012b.h"
#include "c37_torqueBalancing2012b.h"
#include "c40_torqueBalancing2012b.h"
#include "c41_torqueBalancing2012b.h"
#include "c42_torqueBalancing2012b.h"
#include "c43_torqueBalancing2012b.h"
#include "c44_torqueBalancing2012b.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _torqueBalancing2012bMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void torqueBalancing2012b_initializer(void)
{
}

void torqueBalancing2012b_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_torqueBalancing2012b_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==2) {
    c2_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==8) {
    c8_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==9) {
    c9_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==10) {
    c10_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==11) {
    c11_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==12) {
    c12_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==13) {
    c13_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==14) {
    c14_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==15) {
    c15_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==16) {
    c16_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==17) {
    c17_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==18) {
    c18_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==19) {
    c19_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==20) {
    c20_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==21) {
    c21_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==26) {
    c26_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==28) {
    c28_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==29) {
    c29_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==30) {
    c30_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==31) {
    c31_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==32) {
    c32_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==33) {
    c33_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==34) {
    c34_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==35) {
    c35_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==36) {
    c36_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==37) {
    c37_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==40) {
    c40_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==41) {
    c41_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==42) {
    c42_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==43) {
    c43_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==44) {
    c44_torqueBalancing2012b_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_torqueBalancing2012b_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4204367821U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(822064946U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(256938214U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3939742468U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3623743709U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(364483196U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2957002449U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1879759965U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 2:
        {
          extern void sf_c2_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c2_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c3_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c4_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c5_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c6_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c7_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_c8_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c8_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 9:
        {
          extern void sf_c9_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c9_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 10:
        {
          extern void sf_c10_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c10_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 11:
        {
          extern void sf_c11_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c11_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 12:
        {
          extern void sf_c12_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c12_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 13:
        {
          extern void sf_c13_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c13_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 14:
        {
          extern void sf_c14_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c14_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 15:
        {
          extern void sf_c15_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c15_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 16:
        {
          extern void sf_c16_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c16_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 17:
        {
          extern void sf_c17_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c17_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 18:
        {
          extern void sf_c18_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c18_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 19:
        {
          extern void sf_c19_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c19_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 20:
        {
          extern void sf_c20_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c20_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 21:
        {
          extern void sf_c21_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c21_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 26:
        {
          extern void sf_c26_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c26_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 28:
        {
          extern void sf_c28_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c28_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 29:
        {
          extern void sf_c29_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c29_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 30:
        {
          extern void sf_c30_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c30_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 31:
        {
          extern void sf_c31_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c31_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 32:
        {
          extern void sf_c32_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c32_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 33:
        {
          extern void sf_c33_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c33_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 34:
        {
          extern void sf_c34_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c34_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 35:
        {
          extern void sf_c35_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c35_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 36:
        {
          extern void sf_c36_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c36_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 37:
        {
          extern void sf_c37_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c37_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 40:
        {
          extern void sf_c40_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c40_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 41:
        {
          extern void sf_c41_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c41_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 42:
        {
          extern void sf_c42_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c42_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 43:
        {
          extern void sf_c43_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c43_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       case 44:
        {
          extern void sf_c44_torqueBalancing2012b_get_check_sum(mxArray *plhs[]);
          sf_c44_torqueBalancing2012b_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1764838350U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3410240878U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(118138738U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(243351119U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1806387025U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3098901447U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2306735714U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2436158703U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_torqueBalancing2012b_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c2_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c2_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "EX18UBeAltytyKxXaAuhAG") == 0) {
          extern mxArray *sf_c3_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c3_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "zcwXeLXPiG6Bjb6oS6nqFB") == 0) {
          extern mxArray *sf_c4_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c4_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "aGfeCGcwK6wHUSURFtHUY") == 0) {
          extern mxArray *sf_c5_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c5_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "n94yI7eaWsMM62Yc6nXnKH") == 0) {
          extern mxArray *sf_c6_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c6_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "aGfeCGcwK6wHUSURFtHUY") == 0) {
          extern mxArray *sf_c7_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c7_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 8:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c8_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c8_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 9:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c9_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c9_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 10:
      {
        if (strcmp(aiChksum, "UvPxi1ZdnPR7AHUR623VMC") == 0) {
          extern mxArray *sf_c10_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c10_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 11:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c11_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c11_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 12:
      {
        if (strcmp(aiChksum, "SOj1hYgXUf40sPHtPExqMG") == 0) {
          extern mxArray *sf_c12_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c12_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 13:
      {
        if (strcmp(aiChksum, "Wizc3InTzRfgxqurK4FrU") == 0) {
          extern mxArray *sf_c13_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c13_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 14:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c14_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c14_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 15:
      {
        if (strcmp(aiChksum, "XbXLi8aTXx8Iw3BntxBYdB") == 0) {
          extern mxArray *sf_c15_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c15_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 16:
      {
        if (strcmp(aiChksum, "XbXLi8aTXx8Iw3BntxBYdB") == 0) {
          extern mxArray *sf_c16_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c16_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 17:
      {
        if (strcmp(aiChksum, "GllqcgkcvOjLpr8WJJ6qTF") == 0) {
          extern mxArray *sf_c17_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c17_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 18:
      {
        if (strcmp(aiChksum, "OXq7VKrcGTj6ZpLjjYcL4G") == 0) {
          extern mxArray *sf_c18_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c18_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 19:
      {
        if (strcmp(aiChksum, "SOj1hYgXUf40sPHtPExqMG") == 0) {
          extern mxArray *sf_c19_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c19_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 20:
      {
        if (strcmp(aiChksum, "xcFaCjBSgOmhoBADfFt9WF") == 0) {
          extern mxArray *sf_c20_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c20_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 21:
      {
        if (strcmp(aiChksum, "qWpq40tCqAq0Y38jsEMeXB") == 0) {
          extern mxArray *sf_c21_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c21_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 26:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c26_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c26_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 28:
      {
        if (strcmp(aiChksum, "9B1P5X3qhYr65sYSis5Tw") == 0) {
          extern mxArray *sf_c28_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c28_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 29:
      {
        if (strcmp(aiChksum, "9B1P5X3qhYr65sYSis5Tw") == 0) {
          extern mxArray *sf_c29_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c29_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 30:
      {
        if (strcmp(aiChksum, "9B1P5X3qhYr65sYSis5Tw") == 0) {
          extern mxArray *sf_c30_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c30_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 31:
      {
        if (strcmp(aiChksum, "9B1P5X3qhYr65sYSis5Tw") == 0) {
          extern mxArray *sf_c31_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c31_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 32:
      {
        if (strcmp(aiChksum, "9B1P5X3qhYr65sYSis5Tw") == 0) {
          extern mxArray *sf_c32_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c32_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 33:
      {
        if (strcmp(aiChksum, "9B1P5X3qhYr65sYSis5Tw") == 0) {
          extern mxArray *sf_c33_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c33_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 34:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c34_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c34_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 35:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c35_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c35_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 36:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c36_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c36_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 37:
      {
        if (strcmp(aiChksum, "40WA98L0ERNwXhQSEKm7zG") == 0) {
          extern mxArray *sf_c37_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c37_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 40:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c40_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c40_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 41:
      {
        if (strcmp(aiChksum, "JVqb6xiYBG6GmL63tnomKH") == 0) {
          extern mxArray *sf_c41_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c41_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 42:
      {
        if (strcmp(aiChksum, "GAMkWwz11uLc3rQ0ejtJpH") == 0) {
          extern mxArray *sf_c42_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c42_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 43:
      {
        if (strcmp(aiChksum, "7fqLMcXWOY0gah5W9lnSGH") == 0) {
          extern mxArray *sf_c43_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c43_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 44:
      {
        if (strcmp(aiChksum, "Bkebf0FnCzmDSB8D2e5Xx") == 0) {
          extern mxArray *sf_c44_torqueBalancing2012b_get_autoinheritance_info
            (void);
          plhs[0] = sf_c44_torqueBalancing2012b_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_torqueBalancing2012b_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        extern const mxArray
          *sf_c2_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray
          *sf_c7_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 8:
      {
        extern const mxArray
          *sf_c8_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 9:
      {
        extern const mxArray
          *sf_c9_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c9_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 10:
      {
        extern const mxArray
          *sf_c10_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c10_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 11:
      {
        extern const mxArray
          *sf_c11_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c11_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 12:
      {
        extern const mxArray
          *sf_c12_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c12_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 13:
      {
        extern const mxArray
          *sf_c13_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c13_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 14:
      {
        extern const mxArray
          *sf_c14_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c14_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 15:
      {
        extern const mxArray
          *sf_c15_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c15_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 16:
      {
        extern const mxArray
          *sf_c16_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c16_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 17:
      {
        extern const mxArray
          *sf_c17_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c17_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 18:
      {
        extern const mxArray
          *sf_c18_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c18_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 19:
      {
        extern const mxArray
          *sf_c19_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c19_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 20:
      {
        extern const mxArray
          *sf_c20_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c20_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 21:
      {
        extern const mxArray
          *sf_c21_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c21_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 26:
      {
        extern const mxArray
          *sf_c26_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c26_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 28:
      {
        extern const mxArray
          *sf_c28_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c28_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 29:
      {
        extern const mxArray
          *sf_c29_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c29_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 30:
      {
        extern const mxArray
          *sf_c30_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c30_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 31:
      {
        extern const mxArray
          *sf_c31_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c31_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 32:
      {
        extern const mxArray
          *sf_c32_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c32_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 33:
      {
        extern const mxArray
          *sf_c33_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c33_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 34:
      {
        extern const mxArray
          *sf_c34_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c34_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 35:
      {
        extern const mxArray
          *sf_c35_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c35_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 36:
      {
        extern const mxArray
          *sf_c36_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c36_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 37:
      {
        extern const mxArray
          *sf_c37_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c37_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 40:
      {
        extern const mxArray
          *sf_c40_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c40_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 41:
      {
        extern const mxArray
          *sf_c41_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c41_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 42:
      {
        extern const mxArray
          *sf_c42_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c42_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 43:
      {
        extern const mxArray
          *sf_c43_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c43_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 44:
      {
        extern const mxArray
          *sf_c44_torqueBalancing2012b_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c44_torqueBalancing2012b_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_torqueBalancing2012b_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c2_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c2_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "rrzRfmA6jg71OCcou75XRB") == 0) {
          extern mxArray *sf_c3_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c3_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "vZCkkmTRf2hKhmiWECyBEF") == 0) {
          extern mxArray *sf_c4_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c4_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "q9hz9H0nLOBpguDFDiv7VE") == 0) {
          extern mxArray *sf_c5_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c5_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "5CEz2d8DT1mkEZ0bysJblC") == 0) {
          extern mxArray *sf_c6_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c6_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "q9hz9H0nLOBpguDFDiv7VE") == 0) {
          extern mxArray *sf_c7_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c7_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c8_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c8_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c9_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c9_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 10:
      {
        if (strcmp(tpChksum, "n8EtlPdLXBAK10lkieUrlB") == 0) {
          extern mxArray *sf_c10_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c10_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 11:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c11_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c11_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 12:
      {
        if (strcmp(tpChksum, "dUrGMBS4hIM1loakG1cU9B") == 0) {
          extern mxArray *sf_c12_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c12_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 13:
      {
        if (strcmp(tpChksum, "O6Vzd73auY6P08e1tDcUhC") == 0) {
          extern mxArray *sf_c13_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c13_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 14:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c14_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c14_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 15:
      {
        if (strcmp(tpChksum, "QuYTs5VTfnJFG5EPvT3iHD") == 0) {
          extern mxArray *sf_c15_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c15_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 16:
      {
        if (strcmp(tpChksum, "QuYTs5VTfnJFG5EPvT3iHD") == 0) {
          extern mxArray *sf_c16_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c16_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 17:
      {
        if (strcmp(tpChksum, "6nv4CdtaLrs5DniBIz9ceC") == 0) {
          extern mxArray *sf_c17_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c17_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 18:
      {
        if (strcmp(tpChksum, "ceCZAQV4KdwJlTetl0KEwH") == 0) {
          extern mxArray *sf_c18_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c18_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 19:
      {
        if (strcmp(tpChksum, "dUrGMBS4hIM1loakG1cU9B") == 0) {
          extern mxArray *sf_c19_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c19_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 20:
      {
        if (strcmp(tpChksum, "6QWUhsbfRLaDTkqET5sIzD") == 0) {
          extern mxArray *sf_c20_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c20_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 21:
      {
        if (strcmp(tpChksum, "n2hQR67RCIsglVoAkBNkpG") == 0) {
          extern mxArray *sf_c21_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c21_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 26:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c26_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c26_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 28:
      {
        if (strcmp(tpChksum, "UFHwbqPs0saPLmVxuEEcbF") == 0) {
          extern mxArray *sf_c28_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c28_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 29:
      {
        if (strcmp(tpChksum, "UFHwbqPs0saPLmVxuEEcbF") == 0) {
          extern mxArray *sf_c29_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c29_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 30:
      {
        if (strcmp(tpChksum, "UFHwbqPs0saPLmVxuEEcbF") == 0) {
          extern mxArray *sf_c30_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c30_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 31:
      {
        if (strcmp(tpChksum, "UFHwbqPs0saPLmVxuEEcbF") == 0) {
          extern mxArray *sf_c31_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c31_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 32:
      {
        if (strcmp(tpChksum, "UFHwbqPs0saPLmVxuEEcbF") == 0) {
          extern mxArray *sf_c32_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c32_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 33:
      {
        if (strcmp(tpChksum, "UFHwbqPs0saPLmVxuEEcbF") == 0) {
          extern mxArray *sf_c33_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c33_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 34:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c34_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c34_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 35:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c35_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c35_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 36:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c36_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c36_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 37:
      {
        if (strcmp(tpChksum, "Kx9wS4Zz1XwTxccw4ZeqtD") == 0) {
          extern mxArray *sf_c37_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c37_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 40:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c40_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c40_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 41:
      {
        if (strcmp(tpChksum, "bUArhg8cyMaqpRYzORFrKH") == 0) {
          extern mxArray *sf_c41_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c41_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 42:
      {
        if (strcmp(tpChksum, "mMoCWwY2BMjddmLlWvtQ7D") == 0) {
          extern mxArray *sf_c42_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c42_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 43:
      {
        if (strcmp(tpChksum, "8OpXjWnDnjWNIkFSaXaeVH") == 0) {
          extern mxArray *sf_c43_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c43_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     case 44:
      {
        if (strcmp(tpChksum, "FYkoAvaCi6PVZ6s7gqJAdG") == 0) {
          extern mxArray *sf_c44_torqueBalancing2012b_third_party_uses_info(void);
          plhs[0] = sf_c44_torqueBalancing2012b_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void torqueBalancing2012b_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _torqueBalancing2012bMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"torqueBalancing2012b","sfun",0,36,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _torqueBalancing2012bMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _torqueBalancing2012bMachineNumber_,0);
}

void torqueBalancing2012b_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_torqueBalancing2012b_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "torqueBalancing2012b", "torqueBalancing2012b");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_torqueBalancing2012b_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
