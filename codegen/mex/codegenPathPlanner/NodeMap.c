/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * NodeMap.c
 *
 * Code generation for function 'NodeMap'
 *
 */

/* Include files */
#include "NodeMap.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_internal_types.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include "priorityqueue_api.hpp"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo qb_emlrtRTEI = {
    112,                  /* lineNo */
    13,                   /* colNo */
    "NodeMap/insertNode", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m" /* pName */
};

static emlrtRTEInfo yb_emlrtRTEI = {
    102,                 /* lineNo */
    13,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m" /* pName */
};

static emlrtBCInfo ge_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    98,                  /* lineNo */
    41,                  /* colNo */
    "",                  /* aName */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    0                     /* checkKind */
};

static emlrtDCInfo qc_emlrtDCI = {
    98,                  /* lineNo */
    41,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    1                     /* checkKind */
};

static emlrtBCInfo he_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    98,                  /* lineNo */
    39,                  /* colNo */
    "",                  /* aName */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    0                     /* checkKind */
};

static emlrtBCInfo ie_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    98,                  /* lineNo */
    34,                  /* colNo */
    "",                  /* aName */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    0                     /* checkKind */
};

static emlrtBCInfo je_emlrtBCI = {
    -1,                  /* iFirst */
    -1,                  /* iLast */
    98,                  /* lineNo */
    32,                  /* colNo */
    "",                  /* aName */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    0                     /* checkKind */
};

static emlrtRTEInfo ac_emlrtRTEI = {
    97,                  /* lineNo */
    13,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m" /* pName */
};

static emlrtRTEInfo bc_emlrtRTEI = {
    90,                  /* lineNo */
    13,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m" /* pName */
};

static emlrtRTEInfo cc_emlrtRTEI = {
    82,                  /* lineNo */
    13,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m" /* pName */
};

static emlrtDCInfo rc_emlrtDCI = {
    92,                  /* lineNo */
    41,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    1                     /* checkKind */
};

static emlrtDCInfo sc_emlrtDCI = {
    92,                  /* lineNo */
    41,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    4                     /* checkKind */
};

static emlrtDCInfo tc_emlrtDCI = {
    92,                  /* lineNo */
    46,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    1                     /* checkKind */
};

static emlrtDCInfo uc_emlrtDCI = {
    92,                  /* lineNo */
    46,                  /* colNo */
    "NodeMap/traceBack", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m", /* pName */
    4                     /* checkKind */
};

static emlrtRTEInfo ii_emlrtRTEI = {
    92,        /* lineNo */
    35,        /* colNo */
    "NodeMap", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m" /* pName */
};

static emlrtRTEInfo ji_emlrtRTEI = {
    98,        /* lineNo */
    13,        /* colNo */
    "NodeMap", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m" /* pName */
};

static emlrtRTEInfo ki_emlrtRTEI = {
    92,        /* lineNo */
    13,        /* colNo */
    "NodeMap", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\+nav\\+algs\\+internal\\+"
    "codegen\\NodeMap.m" /* pName */
};

/* Function Definitions */
nav_algs_internal_NodeMap *NodeMap_NodeMap(nav_algs_internal_NodeMap *obj)
{
  nav_algs_internal_NodeMap *b_obj;
  b_obj = obj;
  b_obj->NodeMapInternal = NULL;
  b_obj->NodeMapInternal = priorityqueuecodegen_constructNodeMap(3.0);
  b_obj->matlabCodegenIsDeleted = false;
  return b_obj;
}

void NodeMap_insertNode(const emlrtStack *sp, nav_algs_internal_NodeMap *obj,
                        const real_T nodeData[3], real_T parentId)
{
  real_T numNodes;
  numNodes = priorityqueuecodegen_nodemap_getNumNodes(obj->NodeMapInternal);
  if (!(numNodes < rtInf)) {
    emlrtErrorWithMessageIdR2018a(sp, &qb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  priorityqueuecodegen_nodemap_insertNode(obj->NodeMapInternal, &nodeData[0],
                                          parentId);
}

void NodeMap_traceBack(const emlrtStack *sp, nav_algs_internal_NodeMap *obj,
                       real_T idx, emxArray_real_T *nodeDataVec)
{
  emxArray_real_T *data;
  real_T dim;
  real_T maxNumNodes;
  real_T numNodes;
  real_T *data_data;
  real_T *nodeDataVec_data;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  dim = priorityqueuecodegen_nodemap_getDataDim(obj->NodeMapInternal);
  if (!(dim <= 3.0)) {
    emlrtErrorWithMessageIdR2018a(sp, &cc_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  maxNumNodes = priorityqueuecodegen_nodemap_getNumNodes(obj->NodeMapInternal);
  if (!(maxNumNodes < rtInf)) {
    emlrtErrorWithMessageIdR2018a(sp, &bc_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  emxInit_real_T(sp, &data, 2, &ki_emlrtRTEI);
  if (!(dim >= 0.0)) {
    emlrtNonNegativeCheckR2012b(dim, &sc_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (dim != muDoubleScalarFloor(dim)) {
    emlrtIntegerCheckR2012b(dim, &rc_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = data->size[0] * data->size[1];
  data->size[0] = (int32_T)dim;
  emxEnsureCapacity_real_T(sp, data, i, &ii_emlrtRTEI);
  if (!(maxNumNodes >= 0.0)) {
    emlrtNonNegativeCheckR2012b(maxNumNodes, &uc_emlrtDCI, (emlrtConstCTX)sp);
  }
  if (maxNumNodes != (int32_T)muDoubleScalarFloor(maxNumNodes)) {
    emlrtIntegerCheckR2012b(maxNumNodes, &tc_emlrtDCI, (emlrtConstCTX)sp);
  }
  i = data->size[0] * data->size[1];
  data->size[1] = (int32_T)maxNumNodes;
  emxEnsureCapacity_real_T(sp, data, i, &ii_emlrtRTEI);
  data_data = data->data;
  priorityqueuecodegen_nodemap_traceBack(obj->NodeMapInternal, idx,
                                         &data_data[0], &numNodes);
  if (!(numNodes <= maxNumNodes)) {
    emlrtErrorWithMessageIdR2018a(sp, &ac_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (dim < 1.0) {
    loop_ub = 0;
  } else {
    if (data->size[0] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, data->size[0], &je_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if ((int32_T)dim > data->size[0]) {
      emlrtDynamicBoundsCheckR2012b((int32_T)dim, 1, data->size[0],
                                    &ie_emlrtBCI, (emlrtConstCTX)sp);
    }
    loop_ub = (int32_T)dim;
  }
  if (numNodes < 1.0) {
    b_loop_ub = 0;
  } else {
    if (data->size[1] < 1) {
      emlrtDynamicBoundsCheckR2012b(1, 1, data->size[1], &he_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    if (numNodes != (int32_T)muDoubleScalarFloor(numNodes)) {
      emlrtIntegerCheckR2012b(numNodes, &qc_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)numNodes < 1) || ((int32_T)numNodes > data->size[1])) {
      emlrtDynamicBoundsCheckR2012b((int32_T)numNodes, 1, data->size[1],
                                    &ge_emlrtBCI, (emlrtConstCTX)sp);
    }
    b_loop_ub = (int32_T)numNodes;
  }
  i = nodeDataVec->size[0] * nodeDataVec->size[1];
  nodeDataVec->size[0] = b_loop_ub;
  nodeDataVec->size[1] = loop_ub;
  emxEnsureCapacity_real_T(sp, nodeDataVec, i, &ji_emlrtRTEI);
  nodeDataVec_data = nodeDataVec->data;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      nodeDataVec_data[i1 + nodeDataVec->size[0] * i] =
          data_data[i + data->size[0] * i1];
    }
  }
  emxFree_real_T(sp, &data);
  if (!(nodeDataVec->size[0] <= maxNumNodes)) {
    emlrtErrorWithMessageIdR2018a(sp, &yb_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (NodeMap.c) */
