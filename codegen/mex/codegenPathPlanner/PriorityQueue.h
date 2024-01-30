/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * PriorityQueue.h
 *
 * Code generation for function 'PriorityQueue'
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
nav_algs_internal_PriorityQueue *
PriorityQueue_PriorityQueue(nav_algs_internal_PriorityQueue *obj);

boolean_T PriorityQueue_isEmpty(const emlrtStack *sp,
                                nav_algs_internal_PriorityQueue *obj);

void PriorityQueue_top(const emlrtStack *sp,
                       nav_algs_internal_PriorityQueue *obj,
                       real_T nodeData_data[], int32_T nodeData_size[2]);

real_T b_PriorityQueue_top(const emlrtStack *sp,
                           nav_algs_internal_PriorityQueue *obj,
                           real_T nodeData_data[], int32_T nodeData_size[2]);

/* End of code generation (PriorityQueue.h) */
