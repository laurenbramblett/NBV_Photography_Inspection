/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * binaryOccupancyMap.h
 *
 * Code generation for function 'binaryOccupancyMap'
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
void binaryOccupancyMap_inflate(const emlrtStack *sp, binaryOccupancyMap *obj,
                                real_T varargin_1);

void c_binaryOccupancyMap_checkOccup(const emlrtStack *sp,
                                     const binaryOccupancyMap *obj,
                                     const emxArray_real_T *locs,
                                     emxArray_real_T *occupied);

/* End of code generation (binaryOccupancyMap.h) */
