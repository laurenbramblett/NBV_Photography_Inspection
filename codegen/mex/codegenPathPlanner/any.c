/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * any.c
 *
 * Code generation for function 'any'
 *
 */

/* Include files */
#include "any.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Function Definitions */
boolean_T any(const emxArray_boolean_T *x)
{
  int32_T ix;
  const boolean_T *x_data;
  boolean_T exitg1;
  boolean_T y;
  x_data = x->data;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    if (x_data[ix - 1]) {
      y = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return y;
}

boolean_T b_any(const real_T x_data[])
{
  int32_T ix;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix + 1 < 5)) {
    if ((x_data[ix] == 0.0) || muDoubleScalarIsNaN(x_data[ix])) {
      ix++;
    } else {
      y = true;
      exitg1 = true;
    }
  }
  return y;
}

/* End of code generation (any.c) */
