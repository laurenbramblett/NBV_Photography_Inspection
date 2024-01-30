/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * NodeMap.h
 *
 * Code generation for function 'NodeMap'
 *
 */

#pragma once

/* Include files */
#include "codegenPathPlanner_internal_types.h"
#include "codegenPathPlanner_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
nav_algs_internal_NodeMap *NodeMap_NodeMap(nav_algs_internal_NodeMap *obj);

void NodeMap_insertNode(const emlrtStack *sp, nav_algs_internal_NodeMap *obj,
                        const real_T nodeData[3], real_T parentId);

void NodeMap_traceBack(const emlrtStack *sp, nav_algs_internal_NodeMap *obj,
                       real_T idx, emxArray_real_T *nodeDataVec);

/* End of code generation (NodeMap.h) */
