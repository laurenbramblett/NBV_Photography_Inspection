/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_codegenPathPlanner_api.c
 *
 * Code generation for function '_coder_codegenPathPlanner_api'
 *
 */

/* Include files */
#include "_coder_codegenPathPlanner_api.h"
#include "codegenPathPlanner.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo vj_emlrtRTEI = {
    1,                               /* lineNo */
    1,                               /* colNo */
    "_coder_codegenPathPlanner_api", /* fName */
    ""                               /* pName */
};

/* Function Declarations */
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *mapData,
                                   const char_T *identifier))[10000];

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[10000];

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *startPose,
                                   const char_T *identifier))[3];

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3];

static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *infl,
                                 const char_T *identifier);

static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[10000];

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

/* Function Definitions */
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *mapData,
                                   const char_T *identifier))[10000]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[10000];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(mapData), &thisId);
  emlrtDestroyArray(&mapData);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[10000]
{
  real_T(*y)[10000];
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp,
                                   const mxArray *startPose,
                                   const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(startPose), &thisId);
  emlrtDestroyArray(&startPose);
  return y;
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  static const int32_T iv[2] = {0, 0};
  const mxArray *m;
  const mxArray *y;
  const real_T *u_data;
  u_data = u->data;
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m, &u->size[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *infl,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(infl), &thisId);
  emlrtDestroyArray(&infl);
  return y;
}

static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[10000]
{
  static const int32_T dims[2] = {100, 100};
  real_T(*ret)[10000];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[10000])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims[2] = {1, 3};
  real_T(*ret)[3];
  int32_T iv[2];
  boolean_T bv[2] = {false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void codegenPathPlanner_api(codegenPathPlannerStackData *SD,
                            const mxArray *const prhs[4], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  emxArray_real_T *path;
  real_T(*mapData)[10000];
  real_T(*goalPose)[3];
  real_T(*startPose)[3];
  real_T infl;
  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  /* Marshall function inputs */
  mapData = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "mapData");
  startPose = e_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "startPose");
  goalPose = e_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "goalPose");
  infl = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "infl");
  /* Invoke the target function */
  emxInit_real_T(&st, &path, 2, &vj_emlrtRTEI);
  codegenPathPlanner(SD, &st, *mapData, *startPose, *goalPose, infl, path);
  /* Marshall function outputs */
  path->canFreeData = false;
  *plhs = emlrt_marshallOut(path);
  emxFree_real_T(&st, &path);
  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_codegenPathPlanner_api.c) */
