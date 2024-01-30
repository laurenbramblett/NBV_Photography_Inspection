/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sumMatrixIncludeNaN.c
 *
 * Code generation for function 'sumMatrixIncludeNaN'
 *
 */

/* Include files */
#include "sumMatrixIncludeNaN.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
real_T b_sumColumnB(const real_T x[2])
{
  return x[0] + x[1];
}

real_T c_sumColumnB(const real_T x[5])
{
  return (((x[0] + x[1]) + x[2]) + x[3]) + x[4];
}

real_T sumColumnB(const real_T x_data[], int32_T col)
{
  int32_T i0;
  i0 = (col - 1) * 5;
  return (((x_data[i0] + x_data[i0 + 1]) + x_data[i0 + 2]) + x_data[i0 + 3]) +
         x_data[i0 + 4];
}

/* End of code generation (sumMatrixIncludeNaN.c) */
