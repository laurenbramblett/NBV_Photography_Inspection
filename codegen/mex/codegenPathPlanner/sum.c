/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sum.c
 *
 * Code generation for function 'sum'
 *
 */

/* Include files */
#include "sum.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
int32_T sum(const real_T x_data[], const int32_T x_size[2], real_T y_data[])
{
  int32_T xj;
  int32_T y_size;
  if (x_size[0] == 0) {
    y_size = 0;
  } else {
    int32_T scalarLB;
    int32_T vectorUB;
    int32_T vstride;
    vstride = x_size[0];
    y_size = x_size[0];
    memcpy(&y_data[0], &x_data[0], (uint32_T)vstride * sizeof(real_T));
    scalarLB = (x_size[0] / 2) << 1;
    vectorUB = scalarLB - 2;
    for (xj = 0; xj <= vectorUB; xj += 2) {
      __m128d r;
      r = _mm_loadu_pd(&y_data[xj]);
      _mm_storeu_pd(&y_data[xj],
                    _mm_add_pd(r, _mm_loadu_pd(&x_data[vstride + xj])));
    }
    for (xj = scalarLB; xj < vstride; xj++) {
      y_data[xj] += x_data[vstride + xj];
    }
  }
  return y_size;
}

/* End of code generation (sum.c) */
