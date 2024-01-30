/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diff.c
 *
 * Code generation for function 'diff'
 *
 */

/* Include files */
#include "diff.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo oc_emlrtRTEI = {
    51,     /* lineNo */
    19,     /* colNo */
    "diff", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m" /* pName
                                                                         */
};

/* Function Definitions */
int32_T diff(const emlrtStack *sp, const real_T x_data[], int32_T x_size,
             real_T y_data[])
{
  int32_T m;
  int32_T y_size;
  if (x_size == 0) {
    y_size = 0;
  } else {
    m = x_size - 1;
    if (muIntScalarMin_sint32(m, 1) < 1) {
      y_size = 0;
    } else {
      real_T work_data;
      if (x_size == 1) {
        emlrtErrorWithMessageIdR2018a(
            sp, &oc_emlrtRTEI, "Coder:toolbox:autoDimIncompatibility",
            "Coder:toolbox:autoDimIncompatibility", 0);
      }
      y_size = x_size - 1;
      work_data = x_data[0];
      for (m = 2; m <= x_size; m++) {
        real_T tmp2;
        tmp2 = work_data;
        work_data = x_data[m - 1];
        y_data[m - 2] = work_data - tmp2;
      }
    }
  }
  return y_size;
}

/* End of code generation (diff.c) */
