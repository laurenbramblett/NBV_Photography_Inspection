/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * navPath.h
 *
 * Code generation for function 'navPath'
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
navPath *navPath_navPath(const emlrtStack *sp, navPath *obj,
                         stateSpaceSE2 *stateSpace,
                         const emxArray_real_T *states);

/* End of code generation (navPath.h) */
