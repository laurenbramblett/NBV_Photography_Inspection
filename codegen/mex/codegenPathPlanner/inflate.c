/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inflate.c
 *
 * Code generation for function 'inflate'
 *
 */

/* Include files */
#include "inflate.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void b_and(const emlrtStack *sp, emxArray_boolean_T *in1,
           const emxArray_boolean_T *in2, const emxArray_boolean_T *in3)
{
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  const boolean_T *in2_data;
  const boolean_T *in3_data;
  boolean_T *in1_data;
  in3_data = in3->data;
  in2_data = in2->data;
  if (in3->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->size[0];
  }
  i = in1->size[0];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, in1, i, &rd_emlrtRTEI);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_1_0 = (in3->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2_data[i * stride_0_0] && in3_data[i * stride_1_0]);
  }
}

void binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                      const boolean_T in2[10000], const emxArray_int32_T *in3,
                      const emxArray_boolean_T *in4,
                      const emxArray_int32_T *in5)
{
  const int32_T *in3_data;
  const int32_T *in5_data;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  const boolean_T *in4_data;
  boolean_T *in1_data;
  in5_data = in5->data;
  in4_data = in4->data;
  in3_data = in3->data;
  i = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  emxEnsureCapacity_boolean_T(sp, in1, i, &ud_emlrtRTEI);
  if (in5->size[0] == 1) {
    loop_ub = in3->size[1];
  } else {
    loop_ub = in5->size[0];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[1] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, in1, i, &ud_emlrtRTEI);
  in1_data = in1->data;
  stride_0_1 = (in3->size[1] != 1);
  stride_1_1 = (in5->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2[in3_data[i * stride_0_1] - 1] ||
                   in4_data[in5_data[i * stride_1_1]]);
  }
}

void c_and(const emlrtStack *sp, emxArray_boolean_T *in1,
           const emxArray_boolean_T *in2)
{
  emxArray_boolean_T *b_in1;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  const boolean_T *in2_data;
  boolean_T *b_in1_data;
  boolean_T *in1_data;
  in2_data = in2->data;
  in1_data = in1->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_boolean_T(sp, &b_in1, 1, &rd_emlrtRTEI);
  if (in2->size[0] == 1) {
    loop_ub = in1->size[0];
  } else {
    loop_ub = in2->size[0];
  }
  i = b_in1->size[0];
  b_in1->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, b_in1, i, &rd_emlrtRTEI);
  b_in1_data = b_in1->data;
  stride_0_0 = (in1->size[0] != 1);
  stride_1_0 = (in2->size[0] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in1_data[i] = (in1_data[i * stride_0_0] && in2_data[i * stride_1_0]);
  }
  i = in1->size[0];
  in1->size[0] = b_in1->size[0];
  emxEnsureCapacity_boolean_T(sp, in1, i, &rd_emlrtRTEI);
  in1_data = in1->data;
  loop_ub = b_in1->size[0];
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = b_in1_data[i];
  }
  emxFree_boolean_T(sp, &b_in1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (inflate.c) */
