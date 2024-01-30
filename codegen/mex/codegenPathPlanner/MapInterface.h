/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MapInterface.h
 *
 * Code generation for function 'MapInterface'
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
void MapInterface_getParser(const emlrtStack *sp, const binaryOccupancyMap *map,
                            const emxArray_real_T *varargin_1,
                            emxArray_boolean_T *validIdx);

void MapInterface_grid2world(const emlrtStack *sp,
                             const binaryOccupancyMap *obj,
                             const emxArray_real_T *idx, emxArray_real_T *pos);

void MapInterface_world2gridImpl(const binaryOccupancyMap *obj,
                                 const real_T worldXY[2], real_T gridInd[2]);

void b_MapInterface_world2gridImpl(const emlrtStack *sp,
                                   const binaryOccupancyMap *obj,
                                   const emxArray_real_T *worldXY,
                                   emxArray_real_T *gridInd);

/* End of code generation (MapInterface.h) */
