/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateSpaceSE2.h
 *
 * Code generation for function 'stateSpaceSE2'
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
void c_stateSpaceSE2_enforceStateBou(const emlrtStack *sp,
                                     const stateSpaceSE2 *obj,
                                     emxArray_real_T *state);

/* End of code generation (stateSpaceSE2.h) */
