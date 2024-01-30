/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * plannerAStarGrid1.h
 *
 * Code generation for function 'plannerAStarGrid1'
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
void b_plannerAStarGrid_plan(codegenPathPlannerStackData *SD,
                             const emlrtStack *sp,
                             c_nav_algs_internal_plannerASta *obj,
                             const real_T startIn[2], const real_T goalIn[2]);

c_nav_algs_internal_plannerASta *
d_plannerAStarGrid_plannerAStar(c_nav_algs_internal_plannerASta *obj,
                                const real_T map[10000],
                                real_T obstacleThreshold);

/* End of code generation (plannerAStarGrid1.h) */
