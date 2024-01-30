/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * find.c
 *
 * Code generation for function 'find'
 *
 */

/* Include files */
#include "find.h"
#include "codegenPathPlanner_data.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
int32_T eml_find(const emlrtStack *sp, const boolean_T x[10000],
                 int32_T i_data[])
{
  emlrtStack b_st;
  emlrtStack st;
  int32_T iv[2];
  int32_T i_size;
  int32_T ii;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &an_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i_size = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 10000)) {
    if (x[ii]) {
      i_size++;
      i_data[i_size - 1] = ii + 1;
      if (i_size >= 10000) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (i_size < 1) {
    i_size = 0;
  }
  iv[0] = 1;
  iv[1] = i_size;
  b_st.site = &bn_emlrtRSI;
  indexShapeCheck(&b_st, 10000, iv);
  return i_size;
}

/* End of code generation (find.c) */
