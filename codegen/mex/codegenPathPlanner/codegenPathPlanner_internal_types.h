/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * codegenPathPlanner_internal_types.h
 *
 * Code generation for function 'codegenPathPlanner'
 *
 */

#pragma once

/* Include files */
#include "codegenPathPlanner_types.h"
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_nav_algs_internal_NodeMap
#define typedef_nav_algs_internal_NodeMap
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  void *NodeMapInternal;
} nav_algs_internal_NodeMap;
#endif /* typedef_nav_algs_internal_NodeMap */

#ifndef typedef_nav_algs_internal_PriorityQueue
#define typedef_nav_algs_internal_PriorityQueue
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  void *PQInternal;
} nav_algs_internal_PriorityQueue;
#endif /* typedef_nav_algs_internal_PriorityQueue */

#ifndef typedef_cell_wrap_17
#define typedef_cell_wrap_17
typedef struct {
  char_T f1[6];
} cell_wrap_17;
#endif /* typedef_cell_wrap_17 */

#ifndef typedef_c_robotics_core_internal_NameVa
#define typedef_c_robotics_core_internal_NameVa
typedef struct {
  cell_wrap_17 Defaults[1];
  cell_wrap_17 ParsedResults[1];
} c_robotics_core_internal_NameVa;
#endif /* typedef_c_robotics_core_internal_NameVa */

#ifndef typedef_rtDesignRangeCheckInfo
#define typedef_rtDesignRangeCheckInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtDesignRangeCheckInfo;
#endif /* typedef_rtDesignRangeCheckInfo */

#ifndef typedef_rtRunTimeErrorInfo
#define typedef_rtRunTimeErrorInfo
typedef struct {
  int32_T lineNo;
  int32_T colNo;
  const char_T *fName;
  const char_T *pName;
} rtRunTimeErrorInfo;
#endif /* typedef_rtRunTimeErrorInfo */

/* End of code generation (codegenPathPlanner_internal_types.h) */
