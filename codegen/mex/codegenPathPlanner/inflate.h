/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * inflate.h
 *
 * Code generation for function 'inflate'
 *
 */

#pragma once

/* Include files */
#include "codegenPathPlanner_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_and(const emlrtStack *sp, emxArray_boolean_T *in1,
           const emxArray_boolean_T *in2, const emxArray_boolean_T *in3);

void binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                      const boolean_T in2[10000], const emxArray_int32_T *in3,
                      const emxArray_boolean_T *in4,
                      const emxArray_int32_T *in5);

void c_and(const emlrtStack *sp, emxArray_boolean_T *in1,
           const emxArray_boolean_T *in2);

/* End of code generation (inflate.h) */
