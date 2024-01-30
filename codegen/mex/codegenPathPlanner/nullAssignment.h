/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * nullAssignment.h
 *
 * Code generation for function 'nullAssignment'
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
void b_nullAssignment(real_T x_data[], int32_T x_size[2],
                      const boolean_T idx_data[]);

int32_T c_nullAssignment(real_T x_data[], const boolean_T idx_data[]);

int32_T d_nullAssignment(real_T x_data[], const boolean_T idx_data[]);

void nullAssignment(const emlrtStack *sp, emxArray_real_T *x);

/* End of code generation (nullAssignment.h) */
