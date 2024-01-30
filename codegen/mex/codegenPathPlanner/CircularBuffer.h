/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * CircularBuffer.h
 *
 * Code generation for function 'CircularBuffer'
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
void c_CircularBuffer_getValueAtIndi(const emlrtStack *sp,
                                     const e_matlabshared_autonomous_inter *obj,
                                     const emxArray_real_T *indices,
                                     emxArray_boolean_T *values);

/* End of code generation (CircularBuffer.h) */
