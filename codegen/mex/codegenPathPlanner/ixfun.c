/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ixfun.c
 *
 * Code generation for function 'ixfun'
 *
 */

/* Include files */
#include "ixfun.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo xc_emlrtRTEI = {
    225,          /* lineNo */
    23,           /* colNo */
    "expand_max", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m" /* pName
                                                                            */
};

static emlrtRTEInfo yc_emlrtRTEI = {
    225,          /* lineNo */
    23,           /* colNo */
    "expand_min", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m" /* pName
                                                                            */
};

static emlrtRTEInfo ve_emlrtRTEI = {
    234,     /* lineNo */
    20,      /* colNo */
    "ixfun", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m" /* pName
                                                                            */
};

/* Function Definitions */
void expand_max(const emlrtStack *sp, const emxArray_real_T *a,
                const emxArray_real_T *b, emxArray_real_T *c)
{
  const real_T *a_data;
  const real_T *b_data;
  real_T *c_data;
  int32_T k;
  int32_T sak;
  int32_T sbk;
  uint16_T csz_idx_0;
  b_data = b->data;
  a_data = a->data;
  sak = a->size[0];
  sbk = b->size[0];
  if (b->size[0] == 1) {
    csz_idx_0 = (uint16_T)a->size[0];
  } else if (a->size[0] == 1) {
    csz_idx_0 = (uint16_T)b->size[0];
  } else {
    csz_idx_0 = (uint16_T)muIntScalarMin_sint32(sak, sbk);
    if (a->size[0] != b->size[0]) {
      emlrtErrorWithMessageIdR2018a(sp, &xc_emlrtRTEI,
                                    "MATLAB:sizeDimensionsMustMatch",
                                    "MATLAB:sizeDimensionsMustMatch", 0);
    }
  }
  sak = c->size[0] * c->size[1];
  c->size[0] = csz_idx_0;
  c->size[1] = 3;
  emxEnsureCapacity_real_T(sp, c, sak, &ve_emlrtRTEI);
  c_data = c->data;
  if (csz_idx_0 != 0) {
    boolean_T b1;
    boolean_T b_b;
    b_b = (a->size[0] != 1);
    b1 = (b->size[0] != 1);
    for (sbk = 0; sbk < 3; sbk++) {
      sak = c->size[0] - 1;
      for (k = 0; k <= sak; k++) {
        c_data[k + c->size[0] * sbk] =
            muDoubleScalarMax(a_data[b_b * k + a->size[0] * sbk],
                              b_data[b1 * k + b->size[0] * sbk]);
      }
    }
  }
}

void expand_min(const emlrtStack *sp, const emxArray_real_T *a,
                const emxArray_real_T *b, emxArray_real_T *c)
{
  const real_T *a_data;
  const real_T *b_data;
  real_T *c_data;
  int32_T k;
  int32_T sak;
  int32_T sbk;
  uint16_T csz_idx_0;
  b_data = b->data;
  a_data = a->data;
  sak = a->size[0];
  sbk = b->size[0];
  if (b->size[0] == 1) {
    csz_idx_0 = (uint16_T)a->size[0];
  } else if (a->size[0] == 1) {
    csz_idx_0 = (uint16_T)b->size[0];
  } else {
    csz_idx_0 = (uint16_T)muIntScalarMin_sint32(sak, sbk);
    if (a->size[0] != b->size[0]) {
      emlrtErrorWithMessageIdR2018a(sp, &yc_emlrtRTEI,
                                    "MATLAB:sizeDimensionsMustMatch",
                                    "MATLAB:sizeDimensionsMustMatch", 0);
    }
  }
  sak = c->size[0] * c->size[1];
  c->size[0] = csz_idx_0;
  c->size[1] = 3;
  emxEnsureCapacity_real_T(sp, c, sak, &ve_emlrtRTEI);
  c_data = c->data;
  if (csz_idx_0 != 0) {
    boolean_T b1;
    boolean_T b_b;
    b_b = (a->size[0] != 1);
    b1 = (b->size[0] != 1);
    for (sbk = 0; sbk < 3; sbk++) {
      sak = c->size[0] - 1;
      for (k = 0; k <= sak; k++) {
        c_data[k + c->size[0] * sbk] =
            muDoubleScalarMin(a_data[b_b * k + a->size[0] * sbk],
                              b_data[b1 * k + b->size[0] * sbk]);
      }
    }
  }
}

void expand_mod(const emlrtStack *sp, const emxArray_real_T *a,
                emxArray_real_T *c)
{
  const real_T *a_data;
  real_T *c_data;
  int32_T b_k;
  int32_T i;
  int32_T k;
  a_data = a->data;
  i = c->size[0] * c->size[1];
  c->size[0] = a->size[0];
  c->size[1] = 2;
  emxEnsureCapacity_real_T(sp, c, i, &ve_emlrtRTEI);
  c_data = c->data;
  if (a->size[0] != 0) {
    for (k = 0; k < 2; k++) {
      i = c->size[0] - 1;
      for (b_k = 0; b_k <= i; b_k++) {
        c_data[b_k + c->size[0] * k] =
            b_mod(a_data[b_k + a->size[0] * k], 100.0);
      }
    }
  }
}

/* End of code generation (ixfun.c) */
