/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MapInterface.c
 *
 * Code generation for function 'MapInterface'
 *
 */

/* Include files */
#include "MapInterface.h"
#include "any.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include "validatorOccupancyMap.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ah_emlrtRSI = {
    518,                           /* lineNo */
    "MapInterface/world2gridImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo bh_emlrtRSI = {
    519,                           /* lineNo */
    "MapInterface/world2gridImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo fn_emlrtRSI = {
    328,                       /* lineNo */
    "MapInterface/grid2world", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo gn_emlrtRSI = {
    332,                       /* lineNo */
    "MapInterface/grid2world", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo hn_emlrtRSI = {
    500,                           /* lineNo */
    "MapInterface/grid2worldImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo in_emlrtRSI = {
    501,                           /* lineNo */
    "MapInterface/grid2worldImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo jn_emlrtRSI = {
    511,                           /* lineNo */
    "MapInterface/grid2localImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo uo_emlrtRSI = {
    802,                         /* lineNo */
    "MapInterface/getLocations", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo vo_emlrtRSI = {
    885,                             /* lineNo */
    "MapInterface/validatePosition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo wo_emlrtRSI = {
    529,                           /* lineNo */
    "MapInterface/local2gridImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo xo_emlrtRSI = {
    530,                           /* lineNo */
    "MapInterface/local2gridImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo yo_emlrtRSI = {
    539,                           /* lineNo */
    "MapInterface/local2gridImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtRSInfo ap_emlrtRSI = {
    540,                           /* lineNo */
    "MapInterface/local2gridImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

static emlrtECInfo db_emlrtECI = {
    -1,                            /* nDims */
    545,                           /* lineNo */
    13,                            /* colNo */
    "MapInterface/local2gridImpl", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtBCInfo de_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    541,                           /* lineNo */
    25,                            /* colNo */
    "",                            /* aName */
    "MapInterface/local2gridImpl", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m", /* pName */
    0          /* checkKind */
};

static emlrtECInfo sb_emlrtECI = {
    -1,                              /* nDims */
    891,                             /* lineNo */
    24,                              /* colNo */
    "MapInterface/validatePosition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtECInfo tb_emlrtECI = {
    1,                               /* nDims */
    891,                             /* lineNo */
    24,                              /* colNo */
    "MapInterface/validatePosition", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo wf_emlrtRTEI = {
    557,            /* lineNo */
    23,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo xf_emlrtRTEI = {
    332,            /* lineNo */
    13,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo yf_emlrtRTEI = {
    511,            /* lineNo */
    24,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo ag_emlrtRTEI = {
    512,            /* lineNo */
    17,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo bg_emlrtRTEI = {
    511,            /* lineNo */
    13,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo ph_emlrtRTEI = {
    518,            /* lineNo */
    13,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo qh_emlrtRTEI = {
    529,            /* lineNo */
    23,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo rh_emlrtRTEI = {
    529,            /* lineNo */
    49,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo sh_emlrtRTEI = {
    39,    /* lineNo */
    5,     /* colNo */
    "cat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName
                                                                          */
};

static emlrtRTEInfo th_emlrtRTEI = {
    530,            /* lineNo */
    13,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo uh_emlrtRTEI = {
    539,            /* lineNo */
    13,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo vh_emlrtRTEI = {
    545,            /* lineNo */
    28,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo wh_emlrtRTEI = {
    529,            /* lineNo */
    13,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo xh_emlrtRTEI = {
    539,            /* lineNo */
    25,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo ok_emlrtRTEI = {
    891,            /* lineNo */
    24,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo pk_emlrtRTEI = {
    891,            /* lineNo */
    49,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo qk_emlrtRTEI = {
    892,            /* lineNo */
    17,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRTEInfo rk_emlrtRTEI = {
    892,            /* lineNo */
    42,             /* colNo */
    "MapInterface", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pName */
};

static emlrtRSInfo ut_emlrtRSI = {
    891,                             /* lineNo */
    "MapInterface/validatePosition", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapInte"
    "rface.m" /* pathName */
};

/* Function Declarations */
static void MapInterface_grid2localImpl(const emlrtStack *sp,
                                        const binaryOccupancyMap *obj,
                                        const emxArray_real_T *gridInd,
                                        emxArray_real_T *localXY);

static void MapInterface_local2gridImpl(const emlrtStack *sp,
                                        const binaryOccupancyMap *obj,
                                        const emxArray_real_T *localXY,
                                        emxArray_real_T *gridInd);

static void MapInterface_validatePosition(const emlrtStack *sp,
                                          const emxArray_real_T *pos,
                                          const real_T xlimits[2],
                                          const real_T ylimits[2],
                                          emxArray_boolean_T *validPos);

/* Function Definitions */
static void MapInterface_grid2localImpl(const emlrtStack *sp,
                                        const binaryOccupancyMap *obj,
                                        const emxArray_real_T *gridInd,
                                        emxArray_real_T *localXY)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_real_T *varargin_1;
  emxArray_real_T *varargin_2;
  const real_T *gridInd_data;
  real_T xlimit;
  real_T ylimit;
  real_T *localXY_data;
  real_T *varargin_1_data;
  real_T *varargin_2_data;
  int32_T i;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  gridInd_data = gridInd->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &jn_emlrtRSI;
  xlimit = obj->SharedProperties.GridOriginInLocal[0];
  emxInit_real_T(&st, &varargin_1, 1, &yf_emlrtRTEI);
  i = varargin_1->size[0];
  varargin_1->size[0] = gridInd->size[0];
  emxEnsureCapacity_real_T(&st, varargin_1, i, &yf_emlrtRTEI);
  varargin_1_data = varargin_1->data;
  loop_ub = gridInd->size[0];
  ylimit = obj->SharedProperties.GridOriginInLocal[1];
  emxInit_real_T(&st, &varargin_2, 1, &ag_emlrtRTEI);
  i = varargin_2->size[0];
  varargin_2->size[0] = gridInd->size[0];
  emxEnsureCapacity_real_T(&st, varargin_2, i, &ag_emlrtRTEI);
  varargin_2_data = varargin_2->data;
  scalarLB = (gridInd->size[0] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    _mm_storeu_pd(
        &varargin_1_data[i],
        _mm_add_pd(_mm_set1_pd(xlimit),
                   _mm_sub_pd(_mm_loadu_pd(&gridInd_data[i + gridInd->size[0]]),
                              _mm_set1_pd(1.0))));
    _mm_storeu_pd(&varargin_2_data[i],
                  _mm_add_pd(_mm_set1_pd(ylimit),
                             _mm_sub_pd(_mm_set1_pd(100.0),
                                        _mm_loadu_pd(&gridInd_data[i]))));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    varargin_1_data[i] = xlimit + (gridInd_data[i + gridInd->size[0]] - 1.0);
    varargin_2_data[i] = ylimit + (100.0 - gridInd_data[i]);
  }
  b_st.site = &kn_emlrtRSI;
  c_st.site = &ln_emlrtRSI;
  if (varargin_2->size[0] != varargin_1->size[0]) {
    emlrtErrorWithMessageIdR2018a(&c_st, &s_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  i = localXY->size[0] * localXY->size[1];
  localXY->size[0] = varargin_1->size[0];
  localXY->size[1] = 2;
  emxEnsureCapacity_real_T(sp, localXY, i, &bg_emlrtRTEI);
  localXY_data = localXY->data;
  loop_ub = varargin_1->size[0];
  scalarLB = (varargin_1->size[0] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    __m128d r;
    __m128d r1;
    r = _mm_loadu_pd(&varargin_1_data[i]);
    r1 = _mm_set1_pd(0.5);
    _mm_storeu_pd(&localXY_data[i], _mm_add_pd(r, r1));
    r = _mm_loadu_pd(&varargin_2_data[i]);
    _mm_storeu_pd(&localXY_data[i + localXY->size[0]], _mm_add_pd(r, r1));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    localXY_data[i] = varargin_1_data[i] + 0.5;
    localXY_data[i + localXY->size[0]] = varargin_2_data[i] + 0.5;
  }
  emxFree_real_T(sp, &varargin_2);
  emxFree_real_T(sp, &varargin_1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void MapInterface_local2gridImpl(const emlrtStack *sp,
                                        const binaryOccupancyMap *obj,
                                        const emxArray_real_T *localXY,
                                        emxArray_real_T *gridInd)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_boolean_T b_originIdx;
  emxArray_boolean_T *originIdx;
  emxArray_real_T *gridXY;
  emxArray_real_T *varargin_1;
  emxArray_real_T *varargin_2;
  emxArray_real_T *y;
  real_T a[4];
  const real_T *localXY_data;
  real_T xlimit_idx_0;
  real_T ylimit_idx_0;
  real_T *gridInd_data;
  real_T *gridXY_data;
  real_T *varargin_1_data;
  real_T *varargin_2_data;
  int32_T c_originIdx;
  int32_T exponent;
  int32_T i;
  int32_T idx;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T *originIdx_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  localXY_data = localXY->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  xlimit_idx_0 = obj->SharedProperties.GridOriginInLocal[0];
  ylimit_idx_0 = obj->SharedProperties.GridOriginInLocal[1];
  emxInit_real_T(sp, &gridXY, 2, &wh_emlrtRTEI);
  st.site = &wo_emlrtRSI;
  emxInit_real_T(&st, &varargin_1, 1, &qh_emlrtRTEI);
  i = varargin_1->size[0];
  varargin_1->size[0] = localXY->size[0];
  emxEnsureCapacity_real_T(&st, varargin_1, i, &qh_emlrtRTEI);
  varargin_1_data = varargin_1->data;
  idx = localXY->size[0];
  emxInit_real_T(&st, &varargin_2, 1, &rh_emlrtRTEI);
  i = varargin_2->size[0];
  varargin_2->size[0] = localXY->size[0];
  emxEnsureCapacity_real_T(&st, varargin_2, i, &rh_emlrtRTEI);
  varargin_2_data = varargin_2->data;
  scalarLB = (localXY->size[0] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    _mm_storeu_pd(
        &varargin_1_data[i],
        _mm_add_pd(_mm_set1_pd(-ylimit_idx_0),
                   _mm_loadu_pd(&localXY_data[i + localXY->size[0]])));
    _mm_storeu_pd(
        &varargin_2_data[i],
        _mm_add_pd(_mm_set1_pd(-xlimit_idx_0), _mm_loadu_pd(&localXY_data[i])));
  }
  for (i = scalarLB; i < idx; i++) {
    varargin_1_data[i] = -ylimit_idx_0 + localXY_data[i + localXY->size[0]];
    varargin_2_data[i] = -xlimit_idx_0 + localXY_data[i];
  }
  b_st.site = &kn_emlrtRSI;
  c_st.site = &ln_emlrtRSI;
  if (varargin_2->size[0] != varargin_1->size[0]) {
    emlrtErrorWithMessageIdR2018a(&c_st, &s_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  i = gridXY->size[0] * gridXY->size[1];
  gridXY->size[0] = varargin_1->size[0];
  gridXY->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, gridXY, i, &sh_emlrtRTEI);
  gridXY_data = gridXY->data;
  idx = varargin_1->size[0];
  for (i = 0; i < idx; i++) {
    gridXY_data[i] = varargin_1_data[i];
    gridXY_data[i + gridXY->size[0]] = varargin_2_data[i];
  }
  emxFree_real_T(&b_st, &varargin_2);
  st.site = &xo_emlrtRSI;
  b_st.site = &lo_emlrtRSI;
  i = gridInd->size[0] * gridInd->size[1];
  gridInd->size[0] = gridXY->size[0];
  gridInd->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, gridInd, i, &th_emlrtRTEI);
  gridInd_data = gridInd->data;
  scalarLB = gridXY->size[0] << 1;
  for (i = 0; i < scalarLB; i++) {
    gridInd_data[i] = gridXY_data[i];
  }
  for (vectorUB = 0; vectorUB < scalarLB; vectorUB++) {
    gridInd_data[vectorUB] = muDoubleScalarCeil(gridInd_data[vectorUB]);
  }
  a[0] = muDoubleScalarAbs(xlimit_idx_0);
  a[1] = muDoubleScalarAbs(obj->SharedProperties.GridOriginInLocal[0] + 100.0);
  a[2] = muDoubleScalarAbs(ylimit_idx_0);
  a[3] = muDoubleScalarAbs(obj->SharedProperties.GridOriginInLocal[1] + 100.0);
  if (!muDoubleScalarIsNaN(a[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    vectorUB = 2;
    exitg1 = false;
    while ((!exitg1) && (vectorUB < 5)) {
      if (!muDoubleScalarIsNaN(a[vectorUB - 1])) {
        idx = vectorUB;
        exitg1 = true;
      } else {
        vectorUB++;
      }
    }
  }
  if (idx == 0) {
    ylimit_idx_0 = a[0];
  } else {
    ylimit_idx_0 = a[idx - 1];
    i = idx + 1;
    for (vectorUB = i; vectorUB < 5; vectorUB++) {
      xlimit_idx_0 = a[vectorUB - 1];
      if (ylimit_idx_0 < xlimit_idx_0) {
        ylimit_idx_0 = xlimit_idx_0;
      }
    }
  }
  if (muDoubleScalarIsInf(ylimit_idx_0) || muDoubleScalarIsNaN(ylimit_idx_0)) {
    xlimit_idx_0 = rtNaN;
  } else if (ylimit_idx_0 < 4.4501477170144028E-308) {
    xlimit_idx_0 = 4.94065645841247E-324;
  } else {
    frexp(ylimit_idx_0, &exponent);
    xlimit_idx_0 = ldexp(1.0, exponent - 53);
  }
  emxInit_real_T(sp, &y, 2, &xh_emlrtRTEI);
  st.site = &yo_emlrtRSI;
  b_st.site = &mo_emlrtRSI;
  i = y->size[0] * y->size[1];
  y->size[0] = gridXY->size[0];
  y->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, y, i, &eh_emlrtRTEI);
  varargin_2_data = y->data;
  for (vectorUB = 0; vectorUB < scalarLB; vectorUB++) {
    varargin_2_data[vectorUB] = muDoubleScalarAbs(gridXY_data[vectorUB]);
  }
  emxFree_real_T(&b_st, &gridXY);
  emxInit_boolean_T(sp, &originIdx, 2, &uh_emlrtRTEI);
  i = originIdx->size[0] * originIdx->size[1];
  originIdx->size[0] = y->size[0];
  originIdx->size[1] = 2;
  emxEnsureCapacity_boolean_T(sp, originIdx, i, &uh_emlrtRTEI);
  originIdx_data = originIdx->data;
  xlimit_idx_0 *= 2.0;
  for (i = 0; i < scalarLB; i++) {
    originIdx_data[i] = (varargin_2_data[i] < xlimit_idx_0);
  }
  emxFree_real_T(sp, &y);
  b_originIdx = *originIdx;
  c_originIdx = scalarLB;
  b_originIdx.size = &c_originIdx;
  b_originIdx.numDimensions = 1;
  st.site = &ap_emlrtRSI;
  if (any(&b_originIdx)) {
    idx = scalarLB - 1;
    for (scalarLB = 0; scalarLB <= idx; scalarLB++) {
      if (originIdx_data[scalarLB]) {
        i = (gridInd->size[0] << 1) - 1;
        if (scalarLB > i) {
          emlrtDynamicBoundsCheckR2012b(scalarLB, 0, i, &de_emlrtBCI,
                                        (emlrtConstCTX)sp);
        }
        gridInd_data[scalarLB] = 1.0;
      }
    }
  }
  emxFree_boolean_T(sp, &originIdx);
  emlrtSubAssignSizeCheckR2012b(&gridInd->size[0], 1, &gridInd->size[0], 1,
                                &db_emlrtECI, (emlrtCTX)sp);
  i = varargin_1->size[0];
  varargin_1->size[0] = gridInd->size[0];
  emxEnsureCapacity_real_T(sp, varargin_1, i, &vh_emlrtRTEI);
  varargin_1_data = varargin_1->data;
  idx = gridInd->size[0];
  scalarLB = (gridInd->size[0] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    __m128d r;
    r = _mm_loadu_pd(&gridInd_data[i]);
    _mm_storeu_pd(&varargin_1_data[i], _mm_sub_pd(_mm_set1_pd(101.0), r));
  }
  for (i = scalarLB; i < idx; i++) {
    varargin_1_data[i] = 101.0 - gridInd_data[i];
  }
  idx = varargin_1->size[0];
  for (i = 0; i < idx; i++) {
    gridInd_data[i] = varargin_1_data[i];
  }
  emxFree_real_T(sp, &varargin_1);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

static void MapInterface_validatePosition(const emlrtStack *sp,
                                          const emxArray_real_T *pos,
                                          const real_T xlimits[2],
                                          const real_T ylimits[2],
                                          emxArray_boolean_T *validPos)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_boolean_T *r;
  const real_T *pos_data;
  int32_T i;
  int32_T loop_ub;
  boolean_T *r1;
  boolean_T *validPos_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  pos_data = pos->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &vo_emlrtRSI;
  b_st.site = &ab_emlrtRSI;
  if (pos->size[0] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &pb_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:checkOccupancy:expectedNonempty", 3, 4, 19,
        "input number 2, XY,");
  }
  i = validPos->size[0];
  validPos->size[0] = pos->size[0];
  emxEnsureCapacity_boolean_T(sp, validPos, i, &ok_emlrtRTEI);
  validPos_data = validPos->data;
  loop_ub = pos->size[0];
  emxInit_boolean_T(sp, &r, 1, &ok_emlrtRTEI);
  i = r->size[0];
  r->size[0] = pos->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &pk_emlrtRTEI);
  r1 = r->data;
  for (i = 0; i < loop_ub; i++) {
    real_T d;
    d = pos_data[i];
    validPos_data[i] = (d >= xlimits[0]);
    r1[i] = (d <= xlimits[1]);
  }
  if (validPos->size[0] != r->size[0]) {
    emlrtSizeEqCheck1DR2012b(validPos->size[0], r->size[0], &sb_emlrtECI,
                             (emlrtConstCTX)sp);
  }
  loop_ub = validPos->size[0];
  for (i = 0; i < loop_ub; i++) {
    validPos_data[i] = (validPos_data[i] && r1[i]);
  }
  i = r->size[0];
  r->size[0] = pos->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &qk_emlrtRTEI);
  r1 = r->data;
  loop_ub = pos->size[0];
  for (i = 0; i < loop_ub; i++) {
    r1[i] = (pos_data[i + pos->size[0]] >= ylimits[0]);
  }
  if ((validPos->size[0] != r->size[0]) &&
      ((validPos->size[0] != 1) && (r->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(validPos->size[0], r->size[0], &tb_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (validPos->size[0] == r->size[0]) {
    loop_ub = validPos->size[0];
    for (i = 0; i < loop_ub; i++) {
      validPos_data[i] = (validPos_data[i] && r1[i]);
    }
  } else {
    st.site = &ut_emlrtRSI;
    d_and(&st, validPos, r);
    validPos_data = validPos->data;
  }
  i = r->size[0];
  r->size[0] = pos->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &rk_emlrtRTEI);
  r1 = r->data;
  loop_ub = pos->size[0];
  for (i = 0; i < loop_ub; i++) {
    r1[i] = (pos_data[i + pos->size[0]] <= ylimits[1]);
  }
  if ((validPos->size[0] != r->size[0]) &&
      ((validPos->size[0] != 1) && (r->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(validPos->size[0], r->size[0], &tb_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (validPos->size[0] == r->size[0]) {
    loop_ub = validPos->size[0];
    for (i = 0; i < loop_ub; i++) {
      validPos_data[i] = (validPos_data[i] && r1[i]);
    }
  } else {
    st.site = &ut_emlrtRSI;
    d_and(&st, validPos, r);
  }
  emxFree_boolean_T(sp, &r);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void MapInterface_getParser(const emlrtStack *sp, const binaryOccupancyMap *map,
                            const emxArray_real_T *varargin_1,
                            emxArray_boolean_T *validIdx)
{
  emlrtStack b_st;
  emlrtStack st;
  real_T d_map[2];
  real_T e_map[2];
  real_T b_map;
  real_T c_map;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &ug_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_map = map->SharedProperties.LocalOriginInWorld[0] +
          map->SharedProperties.GridOriginInLocal[0];
  c_map = map->SharedProperties.LocalOriginInWorld[1] +
          map->SharedProperties.GridOriginInLocal[1];
  d_map[0] = b_map;
  e_map[0] = c_map;
  d_map[1] = b_map + 100.0;
  e_map[1] = c_map + 100.0;
  b_st.site = &uo_emlrtRSI;
  MapInterface_validatePosition(&b_st, varargin_1, d_map, e_map, validIdx);
}

void MapInterface_grid2world(const emlrtStack *sp,
                             const binaryOccupancyMap *obj,
                             const emxArray_real_T *idx, emxArray_real_T *pos)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *b_pos;
  const real_T *idx_data;
  real_T *b_pos_data;
  real_T *pos_data;
  int32_T i;
  int32_T i1;
  int32_T k;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T exitg1;
  boolean_T p;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  idx_data = idx->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &fn_emlrtRSI;
  b_st.site = &ab_emlrtRSI;
  p = true;
  i = idx->size[0] << 1;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= i - 1)) {
    if ((!muDoubleScalarIsInf(idx_data[k])) &&
        (!muDoubleScalarIsNaN(idx_data[k])) &&
        (muDoubleScalarFloor(idx_data[k]) == idx_data[k])) {
      k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &g_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:grid2world:expectedInteger", 3, 4, 20, "input number 2, idx,");
  }
  b_st.site = &ab_emlrtRSI;
  if (idx->size[0] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &b_st, &pb_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:grid2world:expectedNonempty", 3, 4, 20, "input number 2, idx,");
  }
  st.site = &gn_emlrtRSI;
  b_st.site = &hn_emlrtRSI;
  MapInterface_grid2localImpl(&b_st, obj, idx, pos);
  pos_data = pos->data;
  b_st.site = &in_emlrtRSI;
  emxInit_real_T(&b_st, &b_pos, 2, &wf_emlrtRTEI);
  i = b_pos->size[0] * b_pos->size[1];
  b_pos->size[0] = pos->size[0];
  b_pos->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, b_pos, i, &wf_emlrtRTEI);
  b_pos_data = b_pos->data;
  k = pos->size[0];
  scalarLB = (k / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 <= vectorUB; i1 += 2) {
      __m128d r;
      r = _mm_loadu_pd(&pos_data[i1 + pos->size[0] * i]);
      _mm_storeu_pd(
          &b_pos_data[i1 + b_pos->size[0] * i],
          _mm_add_pd(r,
                     _mm_set1_pd(obj->SharedProperties.LocalOriginInWorld[i])));
    }
    for (i1 = scalarLB; i1 < k; i1++) {
      b_pos_data[i1 + b_pos->size[0] * i] =
          pos_data[i1 + pos->size[0] * i] +
          obj->SharedProperties.LocalOriginInWorld[i];
    }
  }
  i = pos->size[0] * pos->size[1];
  pos->size[0] = b_pos->size[0];
  pos->size[1] = 2;
  emxEnsureCapacity_real_T(&b_st, pos, i, &xf_emlrtRTEI);
  pos_data = pos->data;
  k = b_pos->size[0] << 1;
  for (i = 0; i < k; i++) {
    pos_data[i] = b_pos_data[i];
  }
  emxFree_real_T(&b_st, &b_pos);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void MapInterface_world2gridImpl(const binaryOccupancyMap *obj,
                                 const real_T worldXY[2], real_T gridInd[2])
{
  __m128d r;
  real_T a[4];
  real_T localXY[2];
  real_T gridXY_idx_0;
  real_T gridXY_idx_1;
  real_T xlimit_idx_0;
  real_T ylimit_idx_0;
  int32_T exponent;
  int32_T idx;
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  r = _mm_loadu_pd(&obj->SharedProperties.LocalOriginInWorld[0]);
  _mm_storeu_pd(&localXY[0], _mm_sub_pd(_mm_loadu_pd(&worldXY[0]), r));
  xlimit_idx_0 = obj->SharedProperties.GridOriginInLocal[0];
  ylimit_idx_0 = obj->SharedProperties.GridOriginInLocal[1];
  gridXY_idx_0 = -ylimit_idx_0 + localXY[1];
  gridXY_idx_1 = -xlimit_idx_0 + localXY[0];
  gridInd[0] = muDoubleScalarCeil(gridXY_idx_0);
  gridInd[1] = muDoubleScalarCeil(gridXY_idx_1);
  a[0] = muDoubleScalarAbs(xlimit_idx_0);
  a[1] = muDoubleScalarAbs(obj->SharedProperties.GridOriginInLocal[0] + 100.0);
  a[2] = muDoubleScalarAbs(ylimit_idx_0);
  a[3] = muDoubleScalarAbs(obj->SharedProperties.GridOriginInLocal[1] + 100.0);
  if (!muDoubleScalarIsNaN(a[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!muDoubleScalarIsNaN(a[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ylimit_idx_0 = a[0];
  } else {
    ylimit_idx_0 = a[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      xlimit_idx_0 = a[k - 1];
      if (ylimit_idx_0 < xlimit_idx_0) {
        ylimit_idx_0 = xlimit_idx_0;
      }
    }
  }
  localXY[0] = muDoubleScalarAbs(gridXY_idx_0);
  localXY[1] = muDoubleScalarAbs(gridXY_idx_1);
  if (muDoubleScalarIsInf(ylimit_idx_0) || muDoubleScalarIsNaN(ylimit_idx_0)) {
    xlimit_idx_0 = rtNaN;
  } else if (ylimit_idx_0 < 4.4501477170144028E-308) {
    xlimit_idx_0 = 4.94065645841247E-324;
  } else {
    frexp(ylimit_idx_0, &exponent);
    xlimit_idx_0 = ldexp(1.0, exponent - 53);
  }
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= 1)) {
    if (localXY[k] < xlimit_idx_0 * 2.0) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (y) {
    if (localXY[0] < xlimit_idx_0 * 2.0) {
      gridInd[0] = 1.0;
    }
    if (localXY[1] < xlimit_idx_0 * 2.0) {
      gridInd[1] = 1.0;
    }
  }
  gridInd[0] = 101.0 - gridInd[0];
}

void b_MapInterface_world2gridImpl(const emlrtStack *sp,
                                   const binaryOccupancyMap *obj,
                                   const emxArray_real_T *worldXY,
                                   emxArray_real_T *gridInd)
{
  emlrtStack st;
  emxArray_real_T *localXY;
  const real_T *worldXY_data;
  real_T *localXY_data;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  st.prev = sp;
  st.tls = sp->tls;
  worldXY_data = worldXY->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &ah_emlrtRSI;
  emxInit_real_T(&st, &localXY, 2, &ph_emlrtRTEI);
  i = localXY->size[0] * localXY->size[1];
  localXY->size[0] = worldXY->size[0];
  localXY->size[1] = 2;
  emxEnsureCapacity_real_T(&st, localXY, i, &ph_emlrtRTEI);
  localXY_data = localXY->data;
  loop_ub = worldXY->size[0];
  scalarLB = (loop_ub / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 <= vectorUB; i1 += 2) {
      _mm_storeu_pd(
          &localXY_data[i1 + localXY->size[0] * i],
          _mm_sub_pd(_mm_loadu_pd(&worldXY_data[i1 + worldXY->size[0] * i]),
                     _mm_set1_pd(obj->SharedProperties.LocalOriginInWorld[i])));
    }
    for (i1 = scalarLB; i1 < loop_ub; i1++) {
      localXY_data[i1 + localXY->size[0] * i] =
          worldXY_data[i1 + worldXY->size[0] * i] -
          obj->SharedProperties.LocalOriginInWorld[i];
    }
  }
  st.site = &bh_emlrtRSI;
  MapInterface_local2gridImpl(&st, obj, localXY, gridInd);
  emxFree_real_T(sp, &localXY);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (MapInterface.c) */
