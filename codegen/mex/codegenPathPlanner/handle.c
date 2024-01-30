/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * handle.c
 *
 * Code generation for function 'handle'
 *
 */

/* Include files */
#include "handle.h"
#include "codegenPathPlanner_internal_types.h"
#include "rt_nonfinite.h"
#include "priorityqueue_api.hpp"
#include <string.h>

/* Function Definitions */
void b_handle_matlabCodegenDestructo(const emlrtStack *sp,
                                     nav_algs_internal_PriorityQueue *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    priorityqueuecodegen_destructPQ(obj->PQInternal);
  }
}

void handle_matlabCodegenDestructor(const emlrtStack *sp,
                                    nav_algs_internal_NodeMap *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    priorityqueuecodegen_destructNodeMap(obj->NodeMapInternal);
  }
}

/* End of code generation (handle.c) */
