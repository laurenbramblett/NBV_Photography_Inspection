/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nnz.c
 *
 * Code generation for function 'nnz'
 *
 */

/* Include files */
#include "nnz.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
int32_T intnnz(const real_T s_data[], int32_T s_size)
{
  int32_T k;
  int32_T n;
  n = 0;
  for (k = 0; k < s_size; k++) {
    if (s_data[k] != 0.0) {
      n++;
    }
  }
  return n;
}

/* End of code generation (nnz.c) */
