/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * allOrAny.c
 *
 * Code generation for function 'allOrAny'
 *
 */

/* Include files */
#include "allOrAny.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
boolean_T allOrAny_anonFcn1(const emxArray_boolean_T *x)
{
  int32_T ix;
  const boolean_T *x_data;
  boolean_T exitg1;
  boolean_T varargout_1;
  x_data = x->data;
  varargout_1 = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= x->size[0])) {
    if (x_data[ix - 1]) {
      varargout_1 = true;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  return varargout_1;
}

/* End of code generation (allOrAny.c) */
