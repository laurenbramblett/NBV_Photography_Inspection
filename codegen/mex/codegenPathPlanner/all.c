/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * all.c
 *
 * Code generation for function 'all'
 *
 */

/* Include files */
#include "all.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo eo_emlrtRSI =
    {
        16,    /* lineNo */
        "all", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\all.m" /* pathName
                                                                        */
};

static emlrtRSInfo fo_emlrtRSI =
    {
        139,        /* lineNo */
        "allOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pathName */
};

static emlrtRSInfo go_emlrtRSI =
    {
        143,        /* lineNo */
        "allOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pathName */
};

static emlrtRSInfo to_emlrtRSI =
    {
        13,    /* lineNo */
        "all", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\all.m" /* pathName
                                                                        */
};

static emlrtRSInfo wr_emlrtRSI =
    {
        136,        /* lineNo */
        "allOrAny", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pathName */
};

static emlrtRTEInfo wb_emlrtRTEI = {
    18,                               /* lineNo */
    27,                               /* colNo */
    "eml_int_forloop_overflow_check", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_"
    "overflow_check.m" /* pName */
};

static emlrtRTEInfo xb_emlrtRTEI =
    {
        47,         /* lineNo */
        19,         /* colNo */
        "allOrAny", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pName */
};

static emlrtRTEInfo uc_emlrtRTEI =
    {
        44,         /* lineNo */
        19,         /* colNo */
        "allOrAny", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\allOrAny."
        "m" /* pName */
};

static emlrtRTEInfo vg_emlrtRTEI =
    {
        16,    /* lineNo */
        5,     /* colNo */
        "all", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\all.m" /* pName
                                                                        */
};

/* Function Definitions */
boolean_T all(const boolean_T x[2])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

void b_all(const emlrtStack *sp, const emxArray_boolean_T *x,
           emxArray_boolean_T *y)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T i1;
  int32_T i2;
  int32_T j;
  int32_T outsize_idx_0;
  const boolean_T *x_data;
  boolean_T *y_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  st.site = &eo_emlrtRSI;
  outsize_idx_0 = x->size[0];
  i2 = y->size[0];
  y->size[0] = x->size[0];
  emxEnsureCapacity_boolean_T(&st, y, i2, &vg_emlrtRTEI);
  y_data = y->data;
  for (i2 = 0; i2 < outsize_idx_0; i2++) {
    y_data[i2] = true;
  }
  outsize_idx_0 = x->size[0];
  i2 = x->size[0];
  i1 = 0;
  b_st.site = &fo_emlrtRSI;
  for (j = 0; j < outsize_idx_0; j++) {
    int32_T ix;
    boolean_T exitg1;
    boolean_T overflow;
    i1++;
    i2++;
    b_st.site = &go_emlrtRSI;
    if ((outsize_idx_0 == 0) || (i1 > i2)) {
      overflow = false;
    } else {
      overflow = (i2 > MAX_int32_T - outsize_idx_0);
    }
    if (outsize_idx_0 == 0) {
      emlrtErrorWithMessageIdR2018a(&b_st, &wb_emlrtRTEI,
                                    "Coder:builtins:VectorStride",
                                    "Coder:builtins:VectorStride", 0);
    }
    if (overflow) {
      c_st.site = &wb_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && (ix <= i2)) {
      if (!x_data[ix - 1]) {
        y_data[j] = false;
        exitg1 = true;
      } else {
        ix += outsize_idx_0;
      }
    }
  }
}

boolean_T c_all(const emxArray_boolean_T *x)
{
  int32_T ix;
  const boolean_T *x_data;
  boolean_T exitg1;
  boolean_T y;
  x_data = x->data;
  y = true;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    if (!x_data[ix - 1]) {
      y = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

void d_all(const emlrtStack *sp, const emxArray_boolean_T *x,
           boolean_T y_data[], int32_T y_size[2])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  int32_T b_i;
  int32_T i;
  int32_T i2;
  const boolean_T *x_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  x_data = x->data;
  st.site = &to_emlrtRSI;
  if ((x->size[0] == 0) && (x->size[1] == 0)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &uc_emlrtRTEI, "Coder:toolbox:eml_all_or_any_specialEmpty",
        "Coder:toolbox:eml_all_or_any_specialEmpty", 0);
  }
  if (((x->size[0] != 1) || (x->size[1] != 1)) && (x->size[0] == 1)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &xb_emlrtRTEI,
        "Coder:toolbox:eml_all_or_any_autoDimIncompatibility",
        "Coder:toolbox:eml_all_or_any_autoDimIncompatibility", 0);
  }
  y_size[0] = 1;
  y_size[1] = (uint16_T)x->size[1];
  i2 = (uint16_T)x->size[1];
  for (i = 0; i < i2; i++) {
    y_data[i] = true;
  }
  i = x->size[1];
  i2 = 0;
  b_st.site = &wr_emlrtRSI;
  for (b_i = 0; b_i < i; b_i++) {
    int32_T a_tmp;
    int32_T i1;
    boolean_T exitg1;
    a_tmp = i2 + x->size[0];
    i1 = i2 + 1;
    i2 = a_tmp;
    b_st.site = &go_emlrtRSI;
    if ((i1 <= a_tmp) && (a_tmp > 2147483646)) {
      c_st.site = &wb_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    exitg1 = false;
    while ((!exitg1) && (i1 <= a_tmp)) {
      if (!x_data[i1 - 1]) {
        y_data[b_i] = false;
        exitg1 = true;
      } else {
        i1++;
      }
    }
  }
}

/* End of code generation (all.c) */
