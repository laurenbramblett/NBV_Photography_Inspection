/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * codegenPathPlanner_initialize.c
 *
 * Code generation for function 'codegenPathPlanner_initialize'
 *
 */

/* Include files */
#include "codegenPathPlanner_initialize.h"
#include "_coder_codegenPathPlanner_mex.h"
#include "codegenPathPlanner_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Declarations */
static void codegenPathPlanner_once(void);

/* Function Definitions */
static void codegenPathPlanner_once(void)
{
  mex_InitInfAndNan();
}

void codegenPathPlanner_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtLicenseCheckR2022a(&st, "EMLRT:runTime:MexFunctionNeedsLicense",
                          "navigation_toolbox", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    codegenPathPlanner_once();
  }
}

/* End of code generation (codegenPathPlanner_initialize.c) */
