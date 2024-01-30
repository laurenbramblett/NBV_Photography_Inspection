/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * validateAStarBuiltinCostFunction.c
 *
 * Code generation for function 'validateAStarBuiltinCostFunction'
 *
 */

/* Include files */
#include "validateAStarBuiltinCostFunction.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtBCInfo u_emlrtBCI = {
    -1,                                 /* iFirst */
    -1,                                 /* iLast */
    18,                                 /* lineNo */
    19,                                 /* colNo */
    "",                                 /* aName */
    "validateAStarBuiltinCostFunction", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+internal\\+"
    "validation\\validateAStarBuiltinCostFunction.m", /* pName */
    0                                                 /* checkKind */
};

/* Function Definitions */
real_T c_validateAStarBuiltinCostFunct(const emlrtStack *sp,
                                       char_T strVal_data[],
                                       int32_T strVal_size[2])
{
  static const char_T a[9] = {'E', 'u', 'c', 'l', 'i', 'd', 'e', 'a', 'n'};
  static const char_T b_cv[9] = {'E', 'u', 'c', 'l', 'i', 'd', 'e', 'a', 'n'};
  static const char_T cv1[9] = {'M', 'a', 'n', 'h', 'a', 't', 't', 'a', 'n'};
  static const char_T cv2[9] = {'C', 'h', 'e', 'b', 'y', 's', 'h', 'e', 'v'};
  int32_T b_idx;
  int32_T exitg1;
  int32_T ii;
  int8_T ii_data[4];
  boolean_T x[4];
  boolean_T exitg2;
  strVal_size[0] = 1;
  strVal_size[1] = 9;
  for (b_idx = 0; b_idx < 9; b_idx++) {
    strVal_data[b_idx] = a[b_idx];
  }
  x[0] = false;
  b_idx = 0;
  do {
    exitg1 = 0;
    if (b_idx < 9) {
      if (b_cv[b_idx] != strVal_data[b_idx]) {
        exitg1 = 1;
      } else {
        b_idx++;
      }
    } else {
      x[0] = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  x[1] = false;
  b_idx = 0;
  do {
    exitg1 = 0;
    if (b_idx < 9) {
      if (cv1[b_idx] != strVal_data[b_idx]) {
        exitg1 = 1;
      } else {
        b_idx++;
      }
    } else {
      x[1] = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  x[2] = false;
  b_idx = 0;
  do {
    exitg1 = 0;
    if (b_idx < 9) {
      if (cv2[b_idx] != strVal_data[b_idx]) {
        exitg1 = 1;
      } else {
        b_idx++;
      }
    } else {
      x[2] = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  x[3] = false;
  b_idx = 0;
  ii = 0;
  exitg2 = false;
  while ((!exitg2) && (ii < 4)) {
    if (x[ii]) {
      b_idx++;
      ii_data[b_idx - 1] = (int8_T)(ii + 1);
      if (b_idx >= 4) {
        exitg2 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (b_idx < 1) {
    b_idx = 0;
  }
  if (b_idx < 1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, b_idx, &u_emlrtBCI, (emlrtConstCTX)sp);
  }
  return ii_data[0];
}

/* End of code generation (validateAStarBuiltinCostFunction.c) */
