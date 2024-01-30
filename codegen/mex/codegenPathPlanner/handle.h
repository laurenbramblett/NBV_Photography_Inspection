/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * handle.h
 *
 * Code generation for function 'handle'
 *
 */

#pragma once

/* Include files */
#include "codegenPathPlanner_internal_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void b_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     nav_algs_internal_PriorityQueue *obj);

void handle_matlabCodegenDestructor(const emlrtStack *sp,
                                    nav_algs_internal_NodeMap *obj);

/* End of code generation (handle.h) */
