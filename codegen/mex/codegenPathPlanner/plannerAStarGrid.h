/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * plannerAStarGrid.h
 *
 * Code generation for function 'plannerAStarGrid'
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
plannerAStarGrid *
c_plannerAStarGrid_plannerAStar(const emlrtStack *sp, plannerAStarGrid *obj,
                                binaryOccupancyMap *varargin_1);

real_T plannerAStarGrid_Euclidean(const emlrtStack *sp, const real_T pose1[2],
                                  const real_T pose2[2]);

void plannerAStarGrid_plan(codegenPathPlannerStackData *SD,
                           const emlrtStack *sp, plannerAStarGrid *obj,
                           const real_T start[2], const real_T goal[2]);

/* End of code generation (plannerAStarGrid.h) */
