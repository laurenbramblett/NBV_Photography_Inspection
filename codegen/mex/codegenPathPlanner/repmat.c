/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo co_emlrtRSI = {
    28,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

static emlrtRTEInfo ub_emlrtRTEI = {
    64,                   /* lineNo */
    15,                   /* colNo */
    "assertValidSizeArg", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\assertValidSizeArg.m" /* pName */
};

static emlrtDCInfo fd_emlrtDCI = {
    31,       /* lineNo */
    14,       /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m", /* pName
                                                                          */
    4 /* checkKind */
};

static emlrtRTEInfo ug_emlrtRTEI = {
    59,       /* lineNo */
    28,       /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

static emlrtRTEInfo hi_emlrtRTEI = {
    53,       /* lineNo */
    9,        /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

/* Function Definitions */
void b_repmat(const emlrtStack *sp, real_T varargin_1, emxArray_real_T *b)
{
  emlrtStack st;
  real_T d;
  real_T *b_data;
  int32_T i;
  int32_T loop_ub_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &co_emlrtRSI;
  if ((varargin_1 != muDoubleScalarFloor(varargin_1)) ||
      muDoubleScalarIsInf(varargin_1) || (varargin_1 < -2.147483648E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &vb_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (varargin_1 <= 0.0) {
    d = 0.0;
  } else {
    d = varargin_1;
  }
  if (!(d <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &ub_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  loop_ub_tmp = (int32_T)varargin_1;
  i = b->size[0];
  b->size[0] = (int32_T)varargin_1;
  emxEnsureCapacity_real_T(sp, b, i, &hi_emlrtRTEI);
  b_data = b->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    b_data[i] = -1.0;
  }
}

void c_repmat(const emlrtStack *sp, const real_T a[3], real_T varargin_1,
              emxArray_real_T *b)
{
  emlrtStack st;
  real_T d;
  real_T *b_data;
  int32_T i;
  int32_T ibmat;
  int32_T itilerow;
  int32_T jcol;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &co_emlrtRSI;
  if ((varargin_1 != muDoubleScalarFloor(varargin_1)) ||
      muDoubleScalarIsInf(varargin_1) || (varargin_1 < -2.147483648E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &vb_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (varargin_1 <= 0.0) {
    d = 0.0;
  } else {
    d = varargin_1;
  }
  if (!(d <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &ub_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  if (!(varargin_1 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(varargin_1, &fd_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = (int32_T)varargin_1;
  ibmat = b->size[0] * b->size[1];
  b->size[0] = (int32_T)varargin_1;
  b->size[1] = 3;
  emxEnsureCapacity_real_T(sp, b, ibmat, &ug_emlrtRTEI);
  b_data = b->data;
  for (jcol = 0; jcol < 3; jcol++) {
    ibmat = jcol * (int32_T)varargin_1;
    for (itilerow = 0; itilerow < i; itilerow++) {
      b_data[ibmat + itilerow] = a[jcol];
    }
  }
}

void d_repmat(const emlrtStack *sp, real_T a, real_T varargin_1,
              emxArray_real_T *b)
{
  emlrtStack st;
  real_T d;
  real_T *b_data;
  int32_T i;
  int32_T loop_ub_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &co_emlrtRSI;
  if ((varargin_1 != muDoubleScalarFloor(varargin_1)) ||
      muDoubleScalarIsInf(varargin_1) || (varargin_1 < -2.147483648E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &vb_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (varargin_1 <= 0.0) {
    d = 0.0;
  } else {
    d = varargin_1;
  }
  if (!(d <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &ub_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  if (!(varargin_1 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(varargin_1, &fd_emlrtDCI, (emlrtConstCTX)sp);
  }
  loop_ub_tmp = (int32_T)varargin_1;
  i = b->size[0];
  b->size[0] = (int32_T)varargin_1;
  emxEnsureCapacity_real_T(sp, b, i, &hi_emlrtRTEI);
  b_data = b->data;
  for (i = 0; i < loop_ub_tmp; i++) {
    b_data[i] = a;
  }
}

void e_repmat(const emlrtStack *sp, const real_T a_data[], real_T varargin_2,
              emxArray_real_T *b)
{
  emlrtStack st;
  real_T d;
  real_T *b_data;
  int32_T i;
  int32_T ibtile;
  int32_T jtilecol;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &co_emlrtRSI;
  if ((varargin_2 != muDoubleScalarFloor(varargin_2)) ||
      muDoubleScalarIsInf(varargin_2) || (varargin_2 < -2.147483648E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &vb_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (varargin_2 <= 0.0) {
    d = 0.0;
  } else {
    d = varargin_2;
  }
  if (!(d <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &ub_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  ibtile = b->size[0] * b->size[1];
  b->size[0] = 4;
  i = (int32_T)varargin_2;
  b->size[1] = (int32_T)varargin_2;
  emxEnsureCapacity_real_T(sp, b, ibtile, &ug_emlrtRTEI);
  b_data = b->data;
  for (jtilecol = 0; jtilecol < i; jtilecol++) {
    ibtile = jtilecol << 2;
    b_data[ibtile] = a_data[0];
    b_data[ibtile + 1] = a_data[1];
    b_data[ibtile + 2] = a_data[2];
    b_data[ibtile + 3] = a_data[3];
  }
}

void repmat(const emlrtStack *sp, const real_T a[2], real_T varargin_1,
            emxArray_real_T *b)
{
  emlrtStack st;
  real_T d;
  real_T *b_data;
  int32_T i;
  int32_T ibmat;
  int32_T itilerow;
  int32_T jcol;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &co_emlrtRSI;
  if ((varargin_1 != muDoubleScalarFloor(varargin_1)) ||
      muDoubleScalarIsInf(varargin_1) || (varargin_1 < -2.147483648E+9)) {
    emlrtErrorWithMessageIdR2018a(
        &st, &vb_emlrtRTEI, "Coder:MATLAB:NonIntegerInput",
        "Coder:MATLAB:NonIntegerInput", 4, 12, MIN_int32_T, 12, MAX_int32_T);
  }
  if (varargin_1 <= 0.0) {
    d = 0.0;
  } else {
    d = varargin_1;
  }
  if (!(d <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &ub_emlrtRTEI, "Coder:MATLAB:pmaxsize",
                                  "Coder:MATLAB:pmaxsize", 0);
  }
  i = (int32_T)varargin_1;
  ibmat = b->size[0] * b->size[1];
  b->size[0] = (int32_T)varargin_1;
  b->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b, ibmat, &ug_emlrtRTEI);
  b_data = b->data;
  for (jcol = 0; jcol < 2; jcol++) {
    ibmat = jcol * (int32_T)varargin_1;
    for (itilerow = 0; itilerow < i; itilerow++) {
      b_data[ibmat + itilerow] = a[jcol];
    }
  }
}

/* End of code generation (repmat.c) */
