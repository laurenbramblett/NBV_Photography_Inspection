/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * flipud.c
 *
 * Code generation for function 'flipud'
 *
 */

/* Include files */
#include "flipud.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void flipud(emxArray_real_T *x)
{
  real_T *x_data;
  int32_T b_i;
  int32_T i;
  int32_T j;
  int32_T m;
  int32_T md2;
  x_data = x->data;
  m = x->size[0] - 1;
  i = x->size[1];
  md2 = x->size[0] >> 1;
  for (j = 0; j < i; j++) {
    for (b_i = 0; b_i < md2; b_i++) {
      real_T xtmp;
      int32_T i1;
      xtmp = x_data[b_i + x->size[0] * j];
      i1 = m - b_i;
      x_data[b_i + x->size[0] * j] = x_data[i1 + x->size[0] * j];
      x_data[i1 + x->size[0] * j] = xtmp;
    }
  }
}

/* End of code generation (flipud.c) */
