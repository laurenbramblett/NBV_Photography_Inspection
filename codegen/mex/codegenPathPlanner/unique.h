/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * unique.h
 *
 * Code generation for function 'unique'
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
int32_T b_unique_vector(const emlrtStack *sp, const real_T a_data[],
                        int32_T a_size, real_T b_data[]);

void unique_vector(const emlrtStack *sp, const emxArray_real_T *a,
                   emxArray_real_T *b);

/* End of code generation (unique.h) */
