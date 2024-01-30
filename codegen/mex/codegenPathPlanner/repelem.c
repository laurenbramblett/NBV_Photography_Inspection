/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repelem.c
 *
 * Code generation for function 'repelem'
 *
 */

/* Include files */
#include "repelem.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ft_emlrtRSI = {
    24,        /* lineNo */
    "repelem", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\repelem.m" /* pathName
                                                                          */
};

static emlrtRSInfo gt_emlrtRSI = {
    121,             /* lineNo */
    "repelemMatrix", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\repelem.m" /* pathName
                                                                          */
};

static emlrtRTEInfo wc_emlrtRTEI = {
    19,        /* lineNo */
    23,        /* colNo */
    "repelem", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\repelem.m" /* pName
                                                                          */
};

static emlrtRTEInfo uj_emlrtRTEI = {
    107,       /* lineNo */
    20,        /* colNo */
    "repelem", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\repelem.m" /* pName
                                                                          */
};

/* Function Definitions */
void repelem(const emlrtStack *sp, const real_T x[6], real_T varargin_1,
             emxArray_real_T *y)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T *y_data;
  int32_T i;
  int32_T j;
  int32_T k;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  if (!(muDoubleScalarFloor(varargin_1) == varargin_1)) {
    emlrtErrorWithMessageIdR2018a(sp, &wc_emlrtRTEI,
                                  "MATLAB:repelem:invalidReplications",
                                  "MATLAB:repelem:invalidReplications", 0);
  }
  st.site = &ft_emlrtRSI;
  i = y->size[0] * y->size[1];
  y->size[0] = (int32_T)varargin_1;
  y->size[1] = 6;
  emxEnsureCapacity_real_T(&st, y, i, &uj_emlrtRTEI);
  y_data = y->data;
  if ((int32_T)varargin_1 != 0) {
    int32_T colIdx;
    boolean_T overflow;
    colIdx = -1;
    overflow = ((int32_T)varargin_1 > 2147483646);
    i = (uint16_T)(int32_T)varargin_1;
    for (j = 0; j < 6; j++) {
      colIdx++;
      b_st.site = &gt_emlrtRSI;
      if (overflow) {
        c_st.site = &wb_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (k = 0; k < i; k++) {
        y_data[k + y->size[0] * colIdx] = x[j];
      }
    }
  }
}

/* End of code generation (repelem.c) */
