/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nonzeros.c
 *
 * Code generation for function 'nonzeros'
 *
 */

/* Include files */
#include "nonzeros.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo
    nc_emlrtRTEI =
        {
            20,         /* lineNo */
            9,          /* colNo */
            "nonzeros", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\sparfun\\nonzero"
            "s.m" /* pName */
};

/* Function Definitions */
int32_T nonzeros(const emlrtStack *sp, const real_T s[5], real_T v_data[])
{
  int32_T i;
  int32_T k;
  int32_T v_size;
  v_size = 0;
  for (k = 0; k < 5; k++) {
    if (s[k] != 0.0) {
      v_size++;
    }
  }
  i = 0;
  for (k = 0; k < 5; k++) {
    real_T d;
    d = s[k];
    if (d != 0.0) {
      i++;
      if (i > v_size) {
        emlrtErrorWithMessageIdR2018a(sp, &nc_emlrtRTEI,
                                      "Coder:builtins:AssertionFailed",
                                      "Coder:builtins:AssertionFailed", 0);
      }
      v_data[i - 1] = d;
    }
  }
  return v_size;
}

/* End of code generation (nonzeros.c) */
