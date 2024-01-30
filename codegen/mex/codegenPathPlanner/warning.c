/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * warning.c
 *
 * Code generation for function 'warning'
 *
 */

/* Include files */
#include "warning.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtMCInfo b_emlrtMCI = {
    14,        /* lineNo */
    25,        /* colNo */
    "warning", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\warning.m" /* pName */
};

static emlrtMCInfo c_emlrtMCI = {
    14,        /* lineNo */
    9,         /* colNo */
    "warning", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\warning.m" /* pName */
};

static emlrtRSInfo st_emlrtRSI = {
    14,        /* lineNo */
    "warning", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\coder\\coder\\lib\\+coder\\+"
    "internal\\warning.m" /* pathName */
};

/* Function Declarations */
static void b_feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location);

static const mxArray *feval(const emlrtStack *sp, const mxArray *m1,
                            const mxArray *m2, emlrtMCInfo *location);

/* Function Definitions */
static void b_feval(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  pArrays[0] = m;
  pArrays[1] = m1;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 2, &pArrays[0], "feval",
                        true, location);
}

static const mxArray *feval(const emlrtStack *sp, const mxArray *m1,
                            const mxArray *m2, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m;
  pArrays[0] = m1;
  pArrays[1] = m2;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m, 2, &pArrays[0],
                               "feval", true, location);
}

void warning(const emlrtStack *sp)
{
  static const int32_T iv[2] = {1, 7};
  static const int32_T iv1[2] = {1, 7};
  static const int32_T iv2[2] = {1, 40};
  static const char_T msgID[40] = {
      'n', 'a', 'v', ':', 'n', 'a', 'v', 'a', 'l', 'g', 's', ':', 'p', 'l',
      'a', 'n', 'n', 'e', 'r', 'a', 's', 't', 'a', 'r', 'g', 'r', 'i', 'd',
      ':', 'N', 'o', 'P', 'a', 't', 'h', 'F', 'o', 'u', 'n', 'd'};
  static const char_T b_u[7] = {'m', 'e', 's', 's', 'a', 'g', 'e'};
  static const char_T u[7] = {'w', 'a', 'r', 'n', 'i', 'n', 'g'};
  emlrtStack st;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *m;
  const mxArray *y;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &u[0]);
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 7, m, &b_u[0]);
  emlrtAssign(&b_y, m);
  c_y = NULL;
  m = emlrtCreateCharArray(2, &iv2[0]);
  emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 40, m, &msgID[0]);
  emlrtAssign(&c_y, m);
  st.site = &st_emlrtRSI;
  b_feval(&st, y, feval(&st, b_y, c_y, &b_emlrtMCI), &c_emlrtMCI);
}

/* End of code generation (warning.c) */
