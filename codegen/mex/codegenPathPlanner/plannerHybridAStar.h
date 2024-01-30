/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * plannerHybridAStar.h
 *
 * Code generation for function 'plannerHybridAStar'
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
plannerHybridAStar *
c_plannerHybridAStar_plannerHyb(const emlrtStack *sp, plannerHybridAStar *obj,
                                validatorOccupancyMap *validator);

navPath *plannerHybridAStar_plan(codegenPathPlannerStackData *SD,
                                 const emlrtStack *sp, plannerHybridAStar *obj,
                                 const real_T start[3], const real_T goal[3],
                                 plannerAStarGrid *iobj_0, navPath *iobj_1);

/* End of code generation (plannerHybridAStar.h) */
