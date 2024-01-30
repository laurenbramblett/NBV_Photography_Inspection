/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ReedsSheppBuiltins.h
 *
 * Code generation for function 'ReedsSheppBuiltins'
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
real_T c_ReedsSheppBuiltins_autonomous(const real_T startPose[3],
                                       const real_T goalPose[3],
                                       real_T turningRadius, real_T forwardCost,
                                       real_T reverseCost,
                                       real_T motionLengths[5],
                                       real_T motionTypes[5]);

void d_ReedsSheppBuiltins_autonomous(
    const emlrtStack *sp, const real_T startPose[3], const real_T goalPose[3],
    const emxArray_real_T *samples, real_T turningRadius,
    const real_T segmentsLengths[5], const int32_T segmentsDirections[5],
    const uint32_T segmentsTypes[5], emxArray_real_T *poses);

void e_ReedsSheppBuiltins_autonomous(
    const emlrtStack *sp, const real_T startPose[3], const real_T goalPose[3],
    const emxArray_real_T *samples, real_T turningRadius,
    const real_T segmentsLengths[5], const int32_T segmentsDirections[5],
    const uint32_T segmentsTypes[5], emxArray_real_T *poses,
    emxArray_real_T *directions);

/* End of code generation (ReedsSheppBuiltins.h) */
