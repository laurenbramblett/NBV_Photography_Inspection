/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * PriorityQueue.c
 *
 * Code generation for function 'PriorityQueue'
 *
 */

/* Include files */
#include "PriorityQueue.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_internal_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include "priorityqueue_api.hpp"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo mb_emlrtRTEI = {
    90,                  /* lineNo */
    13,                  /* colNo */
    "PriorityQueue/top", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\PriorityQueue.m" /* pName */
};

static emlrtDCInfo ic_emlrtDCI = {
    92,                  /* lineNo */
    48,                  /* colNo */
    "PriorityQueue/top", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\PriorityQueue.m", /* pName */
    1                           /* checkKind */
};

static emlrtDCInfo jc_emlrtDCI = {
    92,                  /* lineNo */
    48,                  /* colNo */
    "PriorityQueue/top", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\PriorityQueue.m", /* pName */
    4                           /* checkKind */
};

/* Function Definitions */
nav_algs_internal_PriorityQueue *
PriorityQueue_PriorityQueue(nav_algs_internal_PriorityQueue *obj)
{
  nav_algs_internal_PriorityQueue *b_obj;
  b_obj = obj;
  b_obj->PQInternal = NULL;
  b_obj->PQInternal = priorityqueuecodegen_constructPQ(7.0, 0.0);
  b_obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

boolean_T PriorityQueue_isEmpty(const emlrtStack *sp,
                                nav_algs_internal_PriorityQueue *obj)
{
  emlrtStack st;
  real_T b_flag;
  st.prev = sp;
  st.tls = sp->tls;
  b_flag = priorityqueuecodegen_isEmpty(obj->PQInternal);
  st.site = &lm_emlrtRSI;
  if (muDoubleScalarIsNaN(b_flag)) {
    emlrtErrorWithMessageIdR2018a(&st, &t_emlrtRTEI, "MATLAB:nologicalnan",
                                  "MATLAB:nologicalnan", 0);
  }
  return b_flag != 0.0;
}

void PriorityQueue_top(const emlrtStack *sp,
                       nav_algs_internal_PriorityQueue *obj,
                       real_T nodeData_data[], int32_T nodeData_size[2])
{
  real_T dataDim;
  dataDim = priorityqueuecodegen_getDataDim(obj->PQInternal);
  if (!(dataDim <= 5.0)) {
    emlrtErrorWithMessageIdR2018a(sp, &mb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  nodeData_size[0] = 1;
  if (!(dataDim >= 0.0)) {
    emlrtNonNegativeCheckR2012b(dataDim, &jc_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (dataDim != muDoubleScalarFloor(dataDim)) {
    emlrtIntegerCheckR2012b(dataDim, &ic_emlrtDCI, (emlrtConstCTX)sp);
  }
  nodeData_size[1] = (int32_T)dataDim;
  priorityqueuecodegen_top(obj->PQInternal, &nodeData_data[0], &dataDim);
}

real_T b_PriorityQueue_top(const emlrtStack *sp,
                           nav_algs_internal_PriorityQueue *obj,
                           real_T nodeData_data[], int32_T nodeData_size[2])
{
  real_T dataDim;
  real_T nodeId;
  dataDim = priorityqueuecodegen_getDataDim(obj->PQInternal);
  if (!(dataDim <= 7.0)) {
    emlrtErrorWithMessageIdR2018a(sp, &mb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  nodeData_size[0] = 1;
  if (!(dataDim >= 0.0)) {
    emlrtNonNegativeCheckR2012b(dataDim, &jc_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (dataDim != muDoubleScalarFloor(dataDim)) {
    emlrtIntegerCheckR2012b(dataDim, &ic_emlrtDCI, (emlrtConstCTX)sp);
  }
  nodeData_size[1] = (int32_T)dataDim;
  priorityqueuecodegen_top(obj->PQInternal, &nodeData_data[0], &nodeId);
  nodeId++;
  return nodeId;
}

/* End of code generation (PriorityQueue.c) */
