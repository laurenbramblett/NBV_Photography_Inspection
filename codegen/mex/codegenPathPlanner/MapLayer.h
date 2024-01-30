/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MapLayer.h
 *
 * Code generation for function 'MapLayer'
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
void MapLayer_getValueAllImpl(const emlrtStack *sp, binaryOccupancyMap *obj,
                              boolean_T val[10000]);

void c_MapLayer_getValueAtIndicesInt(const emlrtStack *sp,
                                     const binaryOccupancyMap *obj,
                                     const emxArray_real_T *ind,
                                     emxArray_real_T *val);

/* End of code generation (MapLayer.h) */
