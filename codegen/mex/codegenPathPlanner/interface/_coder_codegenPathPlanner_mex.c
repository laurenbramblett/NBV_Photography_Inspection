/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_codegenPathPlanner_mex.c
 *
 * Code generation for function '_coder_codegenPathPlanner_mex'
 *
 */

/* Include files */
#include "_coder_codegenPathPlanner_mex.h"
#include "_coder_codegenPathPlanner_api.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_initialize.h"
#include "codegenPathPlanner_terminate.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void codegenPathPlanner_mexFunction(codegenPathPlannerStackData *SD,
                                    int32_T nlhs, mxArray *plhs[1],
                                    int32_T nrhs, const mxArray *prhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        18, "codegenPathPlanner");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 18,
                        "codegenPathPlanner");
  }
  /* Call the function. */
  codegenPathPlanner_api(SD, prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  codegenPathPlannerStackData *c_codegenPathPlannerStackDataGl = NULL;
  c_codegenPathPlannerStackDataGl =
      (codegenPathPlannerStackData *)emlrtMxCalloc(
          (size_t)1, (size_t)1U * sizeof(codegenPathPlannerStackData));
  mexAtExit(&codegenPathPlanner_atexit);
  /* Module initialization. */
  codegenPathPlanner_initialize();
  /* Dispatch the entry-point. */
  codegenPathPlanner_mexFunction(c_codegenPathPlannerStackDataGl, nlhs, plhs,
                                 nrhs, prhs);
  /* Module termination. */
  codegenPathPlanner_terminate();
  emlrtMxFree(c_codegenPathPlannerStackDataGl);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_codegenPathPlanner_mex.c) */
