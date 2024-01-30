/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * CircularBufferIndex.h
 *
 * Code generation for function 'CircularBufferIndex'
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
void c_CircularBufferIndex_toBaseMat(const emlrtStack *sp,
                                     const d_matlabshared_autonomous_inter *obj,
                                     const emxArray_real_T *b_index,
                                     emxArray_real_T *region);

/* End of code generation (CircularBufferIndex.h) */
