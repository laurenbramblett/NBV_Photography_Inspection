/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * codegenPathPlanner.h
 *
 * Code generation for function 'codegenPathPlanner'
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
void codegenPathPlanner(codegenPathPlannerStackData *SD, const emlrtStack *sp,
                        const real_T mapData[10000], const real_T startPose[3],
                        const real_T goalPose[3], real_T infl,
                        emxArray_real_T *path);

/* End of code generation (codegenPathPlanner.h) */
