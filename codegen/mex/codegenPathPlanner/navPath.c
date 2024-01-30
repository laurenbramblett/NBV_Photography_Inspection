/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * navPath.c
 *
 * Code generation for function 'navPath'
 *
 */

/* Include files */
#include "navPath.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include "stateSpaceSE2.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo at_emlrtRSI = {
    144,               /* lineNo */
    "navPath/navPath", /* fcnName */
    "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\navPath.m" /* pathName
                                                                            */
};

static emlrtRTEInfo rj_emlrtRTEI = {
    144,       /* lineNo */
    17,        /* colNo */
    "navPath", /* fName */
    "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\navPath.m" /* pName
                                                                            */
};

static emlrtRTEInfo sj_emlrtRTEI = {
    180,       /* lineNo */
    13,        /* colNo */
    "navPath", /* fName */
    "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\navPath.m" /* pName
                                                                            */
};

static emlrtRTEInfo tj_emlrtRTEI = {
    130,       /* lineNo */
    17,        /* colNo */
    "navPath", /* fName */
    "C:\\Program Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\navPath.m" /* pName
                                                                            */
};

/* Function Definitions */
navPath *navPath_navPath(const emlrtStack *sp, navPath *obj,
                         stateSpaceSE2 *stateSpace,
                         const emxArray_real_T *states)
{
  emlrtStack st;
  emxArray_real_T *statesInternal;
  navPath *b_obj;
  const real_T *states_data;
  real_T *statesInternal_data;
  int32_T i;
  int32_T loop_ub;
  st.prev = sp;
  st.tls = sp->tls;
  states_data = states->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  b_obj = obj;
  b_obj->StateSpace = stateSpace;
  emxInit_real_T(sp, &statesInternal, 2, &tj_emlrtRTEI);
  i = statesInternal->size[0] * statesInternal->size[1];
  statesInternal->size[0] = states->size[0];
  statesInternal->size[1] = 3;
  emxEnsureCapacity_real_T(sp, statesInternal, i, &rj_emlrtRTEI);
  statesInternal_data = statesInternal->data;
  loop_ub = states->size[0] * 3;
  for (i = 0; i < loop_ub; i++) {
    statesInternal_data[i] = states_data[i];
  }
  st.site = &at_emlrtRSI;
  c_stateSpaceSE2_enforceStateBou(&st, b_obj->StateSpace, statesInternal);
  statesInternal_data = statesInternal->data;
  i = b_obj->StateInternal->size[0] * b_obj->StateInternal->size[1];
  b_obj->StateInternal->size[0] = statesInternal->size[0];
  b_obj->StateInternal->size[1] = 3;
  emxEnsureCapacity_real_T(sp, b_obj->StateInternal, i, &sj_emlrtRTEI);
  loop_ub = statesInternal->size[0] * 3;
  for (i = 0; i < loop_ub; i++) {
    b_obj->StateInternal->data[i] = statesInternal_data[i];
  }
  b_obj->NumStates = statesInternal->size[0];
  emxFree_real_T(sp, &statesInternal);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
  return b_obj;
}

/* End of code generation (navPath.c) */
