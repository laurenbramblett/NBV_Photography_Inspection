/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * codegenPathPlanner_terminate.c
 *
 * Code generation for function 'codegenPathPlanner_terminate'
 *
 */

/* Include files */
#include "codegenPathPlanner_terminate.h"
#include "_coder_codegenPathPlanner_mex.h"
#include "codegenPathPlanner_data.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void codegenPathPlanner_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void codegenPathPlanner_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (codegenPathPlanner_terminate.c) */
