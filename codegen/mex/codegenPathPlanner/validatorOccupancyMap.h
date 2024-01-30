/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * validatorOccupancyMap.h
 *
 * Code generation for function 'validatorOccupancyMap'
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
void c_validatorOccupancyMap_configu(const emlrtStack *sp,
                                     validatorOccupancyMap *obj);

boolean_T c_validatorOccupancyMap_isMotio(const emlrtStack *sp,
                                          validatorOccupancyMap *obj,
                                          const real_T state1[3],
                                          const real_T state2[3]);

boolean_T c_validatorOccupancyMap_isState(const emlrtStack *sp,
                                          validatorOccupancyMap *obj,
                                          const real_T state[3]);

validatorOccupancyMap *c_validatorOccupancyMap_validat(
    const emlrtStack *sp, validatorOccupancyMap *obj, stateSpaceSE2 *varargin_1,
    binaryOccupancyMap *varargin_3);

void d_and(const emlrtStack *sp, emxArray_boolean_T *in1,
           const emxArray_boolean_T *in2);

void d_validatorOccupancyMap_isState(const emlrtStack *sp,
                                     validatorOccupancyMap *obj,
                                     const emxArray_real_T *state,
                                     emxArray_boolean_T *isValid);

/* End of code generation (validatorOccupancyMap.h) */
