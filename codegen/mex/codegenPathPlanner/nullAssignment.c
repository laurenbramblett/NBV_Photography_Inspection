/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nullAssignment.c
 *
 * Code generation for function 'nullAssignment'
 *
 */

/* Include files */
#include "nullAssignment.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo qr_emlrtRSI = {
    29,               /* lineNo */
    "nullAssignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pathName */
};

static emlrtRSInfo rr_emlrtRSI = {
    33,               /* lineNo */
    "nullAssignment", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pathName */
};

static emlrtRTEInfo cd_emlrtRTEI = {
    378,              /* lineNo */
    1,                /* colNo */
    "delete_columns", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pName */
};

static emlrtRTEInfo sk_emlrtRTEI = {
    33,               /* lineNo */
    13,               /* colNo */
    "nullAssignment", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\nullAssignment.m" /* pName */
};

/* Function Definitions */
void b_nullAssignment(real_T x_data[], int32_T x_size[2],
                      const boolean_T idx_data[])
{
  int32_T i;
  int32_T k;
  int32_T n;
  n = ((idx_data[0] + idx_data[1]) + idx_data[2]) + idx_data[3];
  i = 0;
  for (k = 0; k < 4; k++) {
    if (!idx_data[k]) {
      x_data[i] = x_data[k];
      x_data[i + x_size[0]] = x_data[k + x_size[0]];
      x_data[i + x_size[0] * 2] = x_data[k + x_size[0] * 2];
      i++;
    }
  }
  if (4 - n < 1) {
    n = 0;
  } else {
    n = 4 - n;
  }
  for (i = 0; i < 3; i++) {
    for (k = 0; k < n; k++) {
      x_data[k + n * i] = x_data[k + x_size[0] * i];
    }
  }
  x_size[0] = n;
  x_size[1] = 3;
}

int32_T c_nullAssignment(real_T x_data[], const boolean_T idx_data[])
{
  int32_T i;
  int32_T n;
  n = ((idx_data[0] + idx_data[1]) + idx_data[2]) + idx_data[3];
  i = 0;
  if (!idx_data[0]) {
    i = 1;
  }
  if (!idx_data[1]) {
    x_data[i] = x_data[1];
    i++;
  }
  if (!idx_data[2]) {
    x_data[i] = x_data[2];
    i++;
  }
  if (!idx_data[3]) {
    x_data[i] = x_data[3];
  }
  if (4 - n < 1) {
    n = -1;
  } else {
    n = 3 - n;
  }
  return n + 1;
}

int32_T d_nullAssignment(real_T x_data[], const boolean_T idx_data[])
{
  int32_T k0;
  int32_T n;
  n = ((idx_data[0] + idx_data[1]) + idx_data[2]) + idx_data[3];
  k0 = -1;
  if (!idx_data[0]) {
    k0 = 0;
  }
  if (!idx_data[1]) {
    k0++;
    x_data[k0] = x_data[1];
  }
  if (!idx_data[2]) {
    k0++;
    x_data[k0] = x_data[2];
  }
  if (!idx_data[3]) {
    k0++;
    x_data[k0] = x_data[3];
  }
  if (4 - n < 1) {
    n = -1;
  } else {
    n = 3 - n;
  }
  return n + 1;
}

void nullAssignment(const emlrtStack *sp, emxArray_real_T *x)
{
  emlrtStack st;
  real_T *x_data;
  int32_T i;
  int32_T j;
  int32_T ncols;
  st.prev = sp;
  st.tls = sp->tls;
  x_data = x->data;
  st.site = &qr_emlrtRSI;
  if (x->size[1] < 1) {
    emlrtErrorWithMessageIdR2018a(&st, &hc_emlrtRTEI,
                                  "MATLAB:subsdeldimmismatch",
                                  "MATLAB:subsdeldimmismatch", 0);
  }
  st.site = &rr_emlrtRSI;
  ncols = x->size[1];
  for (j = 0; j <= ncols - 2; j++) {
    i = 4 * (j + 1);
    x_data[4 * j] = x_data[i];
    x_data[4 * j + 1] = x_data[i + 1];
    x_data[4 * j + 2] = x_data[i + 2];
    x_data[4 * j + 3] = x_data[i + 3];
  }
  if (x->size[1] - 1 > x->size[1]) {
    emlrtErrorWithMessageIdR2018a(&st, &cd_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  i = x->size[0] * x->size[1];
  x->size[0] = 4;
  if (x->size[1] - 1 < 1) {
    x->size[1] = 0;
  } else {
    x->size[1]--;
  }
  emxEnsureCapacity_real_T(&st, x, i, &sk_emlrtRTEI);
}

/* End of code generation (nullAssignment.c) */
