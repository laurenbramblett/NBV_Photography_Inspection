/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * all.h
 *
 * Code generation for function 'all'
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
boolean_T all(const boolean_T x[2]);

void b_all(const emlrtStack *sp, const emxArray_boolean_T *x,
           emxArray_boolean_T *y);

boolean_T c_all(const emxArray_boolean_T *x);

void d_all(const emlrtStack *sp, const emxArray_boolean_T *x,
           boolean_T y_data[], int32_T y_size[2]);

/* End of code generation (all.h) */
