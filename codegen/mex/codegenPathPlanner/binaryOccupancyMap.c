/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * binaryOccupancyMap.c
 *
 * Code generation for function 'binaryOccupancyMap'
 *
 */

/* Include files */
#include "binaryOccupancyMap.h"
#include "MapInterface.h"
#include "MapLayer.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "diskstrel.h"
#include "inflate.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo jb_emlrtRSI = {
    360,                          /* lineNo */
    "binaryOccupancyMap/inflate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo kb_emlrtRSI = {
    359,                          /* lineNo */
    "binaryOccupancyMap/inflate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo lb_emlrtRSI = {
    361,                          /* lineNo */
    "binaryOccupancyMap/inflate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pathName */
};

static emlrtRSInfo xb_emlrtRSI = {
    517,                    /* lineNo */
    "MapUtils/inflateGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\MapUtils.m" /* pathName */
};

static emlrtRSInfo yb_emlrtRSI = {
    520,                    /* lineNo */
    "MapUtils/inflateGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\MapUtils.m" /* pathName */
};

static emlrtRSInfo ac_emlrtRSI = {
    521,                    /* lineNo */
    "MapUtils/inflateGrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\MapUtils.m" /* pathName */
};

static emlrtRSInfo bc_emlrtRSI = {
    414,                                /* lineNo */
    "MapUtils/validateInflationRadius", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\MapUtils.m" /* pathName */
};

static emlrtRSInfo cc_emlrtRSI = {
    424,                                /* lineNo */
    "MapUtils/validateInflationRadius", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\MapUtils.m" /* pathName */
};

static emlrtRSInfo nc_emlrtRSI = {
    22,        /* lineNo */
    "inflate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\inflate.m" /* pathName */
};

static emlrtRSInfo oc_emlrtRSI = {
    24,        /* lineNo */
    "inflate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pathName */
};

static emlrtRSInfo pc_emlrtRSI = {
    46,        /* lineNo */
    "inflate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pathName */
};

static emlrtRSInfo qc_emlrtRSI = {
    19,        /* lineNo */
    "ind2sub", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\ind2sub.m" /* pathName
                                                                          */
};

static emlrtRSInfo rc_emlrtRSI = {
    16,        /* lineNo */
    "sub2ind", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\sub2ind.m" /* pathName
                                                                          */
};

static emlrtRTEInfo
    c_emlrtRTEI =
        {
            28,        /* lineNo */
            19,        /* colNo */
            "sub2ind", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sub2ind.m" /* pName */
};

static emlrtRTEInfo
    d_emlrtRTEI =
        {
            18,        /* lineNo */
            23,        /* colNo */
            "sub2ind", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sub2ind.m" /* pName */
};

static emlrtRTEInfo
    e_emlrtRTEI =
        {
            21,        /* lineNo */
            15,        /* colNo */
            "ind2sub", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\ind2sub.m" /* pName */
};

static emlrtRTEInfo i_emlrtRTEI = {
    14,                 /* lineNo */
    37,                 /* colNo */
    "validatepositive", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\+"
    "valattr\\validatepositive.m" /* pName */
};

static emlrtECInfo emlrtECI = {
    -1,        /* nDims */
    56,        /* lineNo */
    17,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtECInfo b_emlrtECI = {
    2,         /* nDims */
    56,        /* lineNo */
    38,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtECInfo c_emlrtECI = {
    1,         /* nDims */
    41,        /* lineNo */
    20,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,        /* iFirst */
    -1,        /* iLast */
    47,        /* lineNo */
    43,        /* colNo */
    "",        /* aName */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,        /* iFirst */
    -1,        /* iLast */
    47,        /* lineNo */
    64,        /* colNo */
    "",        /* aName */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = {
    -1,        /* iFirst */
    -1,        /* iLast */
    55,        /* lineNo */
    28,        /* colNo */
    "",        /* aName */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m", /* pName */
    0                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = {
    1,         /* iFirst */
    10000,     /* iLast */
    56,        /* lineNo */
    50,        /* colNo */
    "",        /* aName */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m", /* pName */
    0                                    /* checkKind */
};

static emlrtRTEInfo
    gd_emlrtRTEI =
        {
            13,        /* lineNo */
            1,         /* colNo */
            "ind2sub", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\ind2sub.m" /* pName */
};

static emlrtRTEInfo
    hd_emlrtRTEI =
        {
            23,        /* lineNo */
            1,         /* colNo */
            "ind2sub", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\ind2sub.m" /* pName */
};

static emlrtRTEInfo
    id_emlrtRTEI =
        {
            25,        /* lineNo */
            5,         /* colNo */
            "ind2sub", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\ind2sub.m" /* pName */
};

static emlrtRTEInfo
    jd_emlrtRTEI =
        {
            27,        /* lineNo */
            5,         /* colNo */
            "ind2sub", /* fName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\ind2sub.m" /* pName */
};

static emlrtRTEInfo kd_emlrtRTEI = {
    21,        /* lineNo */
    5,         /* colNo */
    "ind2sub", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\ind2sub.m" /* pName
                                                                          */
};

static emlrtRTEInfo ld_emlrtRTEI = {
    37,        /* lineNo */
    13,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo md_emlrtRTEI = {
    38,        /* lineNo */
    13,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo nd_emlrtRTEI = {
    41,        /* lineNo */
    21,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo od_emlrtRTEI = {
    42,        /* lineNo */
    21,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo pd_emlrtRTEI = {
    43,        /* lineNo */
    21,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo qd_emlrtRTEI = {
    44,        /* lineNo */
    21,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo sd_emlrtRTEI = {
    328,                  /* lineNo */
    18,                   /* colNo */
    "binaryOccupancyMap", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_"
    "lib\\binaryOccupancyMap.m" /* pName */
};

static emlrtRTEInfo td_emlrtRTEI = {
    46,        /* lineNo */
    13,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo vd_emlrtRTEI = {
    520,        /* lineNo */
    13,         /* colNo */
    "MapUtils", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\MapUtils.m" /* pName */
};

static emlrtRTEInfo wd_emlrtRTEI = {
    56,        /* lineNo */
    17,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo xd_emlrtRTEI = {
    47,        /* lineNo */
    43,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo yd_emlrtRTEI = {
    24,        /* lineNo */
    42,        /* colNo */
    "inflate", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pName */
};

static emlrtRTEInfo oh_emlrtRTEI = {
    1512,       /* lineNo */
    13,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRSInfo tt_emlrtRSI = {
    41,        /* lineNo */
    "inflate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pathName */
};

static emlrtRSInfo xt_emlrtRSI =
    {
        19,            /* lineNo */
        "indexDivide", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\indexDivide.m" /* pathName */
};

static emlrtRSInfo yt_emlrtRSI = {
    56,        /* lineNo */
    "inflate", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\+impl\\inflate.m" /* pathName */
};

/* Function Declarations */
static int32_T div_s32(const emlrtStack *sp, int32_T numerator,
                       int32_T denominator);

/* Function Definitions */
static int32_T div_s32(const emlrtStack *sp, int32_T numerator,
                       int32_T denominator)
{
  int32_T quotient;
  if (denominator == 0) {
    emlrtDivisionByZeroErrorR2012b(NULL, (emlrtConstCTX)sp);
  } else {
    uint32_T tempAbsQuotient;
    uint32_T u;
    if (numerator < 0) {
      tempAbsQuotient = ~(uint32_T)numerator + 1U;
    } else {
      tempAbsQuotient = (uint32_T)numerator;
    }
    if (denominator < 0) {
      u = ~(uint32_T)denominator + 1U;
    } else {
      u = (uint32_T)denominator;
    }
    tempAbsQuotient /= u;
    if ((numerator < 0) != (denominator < 0)) {
      quotient = -(int32_T)tempAbsQuotient;
    } else {
      quotient = (int32_T)tempAbsQuotient;
    }
  }
  return quotient;
}

void binaryOccupancyMap_inflate(const emlrtStack *sp, binaryOccupancyMap *obj,
                                real_T varargin_1)
{
  static const int32_T offsets[4] = {0, 1, 2, 3};
  __m128i r;
  __m128i r1;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack st;
  emxArray_boolean_T *r2;
  emxArray_boolean_T *r3;
  emxArray_boolean_T *r4;
  emxArray_boolean_T *r6;
  emxArray_boolean_T *se;
  emxArray_int32_T *b_index;
  emxArray_int32_T *colIdx;
  emxArray_int32_T *r5;
  emxArray_int32_T *vk;
  emxArray_int32_T *y;
  emxArray_real_T *shiftedColIdx;
  emxArray_real_T *shiftedRowIdx;
  real_T radius;
  real_T *shiftedColIdx_data;
  real_T *shiftedRowIdx_data;
  int32_T iv[4];
  int32_T b_i;
  int32_T b_varargin_1;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T j;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T *colIdx_data;
  int32_T *index_data;
  int32_T *vk_data;
  int32_T *y_data;
  uint32_T siz[2];
  boolean_T inflatedGrid[10000];
  boolean_T map[10000];
  boolean_T exitg1;
  boolean_T *r7;
  boolean_T *r8;
  boolean_T *r9;
  boolean_T *se_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &kb_emlrtRSI;
  b_st.site = &jb_emlrtRSI;
  MapLayer_getValueAllImpl(&b_st, obj, inflatedGrid);
  b_st.site = &xb_emlrtRSI;
  c_st.site = &bc_emlrtRSI;
  d_st.site = &ab_emlrtRSI;
  if (varargin_1 <= 0.0) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &i_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedPositive",
        "MATLAB:inflate:expectedPositive", 3, 4, 1, "R");
  }
  d_st.site = &ab_emlrtRSI;
  if (muDoubleScalarIsNaN(varargin_1)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &b_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedNonNaN",
        "MATLAB:inflate:expectedNonNaN", 3, 4, 1, "R");
  }
  d_st.site = &ab_emlrtRSI;
  if (muDoubleScalarIsInf(varargin_1)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &h_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedFinite",
        "MATLAB:inflate:expectedFinite", 3, 4, 1, "R");
  }
  radius = muDoubleScalarCeil(varargin_1);
  c_st.site = &cc_emlrtRSI;
  d_st.site = &ab_emlrtRSI;
  if (muDoubleScalarIsInf(radius)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &g_emlrtRTEI, "Coder:toolbox:ValidateattributesexpectedInteger",
        "MATLAB:inflate:expectedInteger", 3, 4, 1, "R");
  }
  d_st.site = &ab_emlrtRSI;
  if (!(radius <= 100.0)) {
    emlrtErrorWithMessageIdR2018a(
        &d_st, &f_emlrtRTEI, "MATLAB:validateattributes:expectedArray",
        "MATLAB:inflate:notLessEqual", 9, 4, 1, "R", 4, 2, "<=", 4, 3, "100");
  }
  emxInit_boolean_T(&st, &se, 2, &vd_emlrtRTEI);
  b_st.site = &yb_emlrtRSI;
  diskstrel(&b_st, radius, se);
  se_data = se->data;
  b_st.site = &ac_emlrtRSI;
  c_st.site = &nc_emlrtRSI;
  memcpy(&map[0], &inflatedGrid[0], 10000U * sizeof(boolean_T));
  i = (int32_T)muDoubleScalarCeil((real_T)se->size[0] / 2.0);
  i1 = (int32_T)muDoubleScalarCeil((real_T)se->size[1] / 2.0);
  b_varargin_1 = se->size[0] * se->size[1];
  emxInit_int32_T(&c_st, &y, 2, &yd_emlrtRTEI);
  y_data = y->data;
  if (b_varargin_1 < 1) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    i2 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = b_varargin_1;
    emxEnsureCapacity_int32_T(&c_st, y, i2, &fd_emlrtRTEI);
    y_data = y->data;
    scalarLB = (b_varargin_1 / 4) << 2;
    vectorUB = scalarLB - 4;
    for (i2 = 0; i2 <= vectorUB; i2 += 4) {
      _mm_storeu_si128(
          (__m128i *)&y_data[i2],
          _mm_add_epi32(
              _mm_set1_epi32(1),
              _mm_add_epi32(_mm_set1_epi32(i2),
                            _mm_loadu_si128((const __m128i *)&offsets[0]))));
    }
    for (i2 = scalarLB; i2 < b_varargin_1; i2++) {
      y_data[i2] = i2 + 1;
    }
  }
  d_st.site = &oc_emlrtRSI;
  siz[0] = (uint32_T)se->size[0];
  e_st.site = &qc_emlrtRSI;
  emxInit_int32_T(&e_st, &b_index, 2, &td_emlrtRTEI);
  i2 = b_index->size[0] * b_index->size[1];
  b_index->size[0] = 1;
  b_index->size[1] = y->size[1];
  emxEnsureCapacity_int32_T(&e_st, b_index, i2, &gd_emlrtRTEI);
  index_data = b_index->data;
  loop_ub = y->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    index_data[i2] = y_data[i2];
  }
  b_varargin_1 = se->size[0] * se->size[1];
  scalarLB = 0;
  exitg1 = false;
  while ((!exitg1) && (scalarLB <= b_index->size[1] - 1)) {
    if (index_data[scalarLB] > b_varargin_1) {
      emlrtErrorWithMessageIdR2018a(&e_st, &e_emlrtRTEI,
                                    "Coder:MATLAB:ind2sub_IndexOutOfRange",
                                    "Coder:MATLAB:ind2sub_IndexOutOfRange", 0);
    } else {
      scalarLB++;
    }
  }
  i2 = b_index->size[0] * b_index->size[1];
  b_index->size[0] = 1;
  emxEnsureCapacity_int32_T(&e_st, b_index, i2, &hd_emlrtRTEI);
  index_data = b_index->data;
  loop_ub = b_index->size[1] - 1;
  scalarLB = (b_index->size[1] / 4) << 2;
  vectorUB = scalarLB - 4;
  for (i2 = 0; i2 <= vectorUB; i2 += 4) {
    r = _mm_loadu_si128((const __m128i *)&index_data[i2]);
    _mm_storeu_si128((__m128i *)&index_data[i2],
                     _mm_sub_epi32(r, _mm_set1_epi32(1)));
  }
  for (i2 = scalarLB; i2 <= loop_ub; i2++) {
    index_data[i2]--;
  }
  emxInit_int32_T(&e_st, &vk, 2, &id_emlrtRTEI);
  i2 = vk->size[0] * vk->size[1];
  vk->size[0] = 1;
  vk->size[1] = b_index->size[1];
  emxEnsureCapacity_int32_T(&e_st, vk, i2, &id_emlrtRTEI);
  vk_data = vk->data;
  loop_ub = b_index->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    f_st.site = &xt_emlrtRSI;
    vk_data[i2] = div_s32(&f_st, index_data[i2], (int32_T)siz[0]);
  }
  i2 = b_index->size[0] * b_index->size[1];
  b_index->size[0] = 1;
  emxEnsureCapacity_int32_T(&e_st, b_index, i2, &jd_emlrtRTEI);
  index_data = b_index->data;
  b_varargin_1 = se->size[0];
  loop_ub = b_index->size[1] - 1;
  scalarLB = (b_index->size[1] / 4) << 2;
  vectorUB = scalarLB - 4;
  for (i2 = 0; i2 <= vectorUB; i2 += 4) {
    iv[0] = vk_data[i2] * b_varargin_1;
    iv[1] = vk_data[i2 + 1] * b_varargin_1;
    iv[2] = vk_data[i2 + 2] * b_varargin_1;
    iv[3] = vk_data[i2 + 3] * b_varargin_1;
    r = _mm_loadu_si128((const __m128i *)&index_data[i2]);
    r1 = _mm_loadu_si128((const __m128i *)&iv[0]);
    _mm_storeu_si128((__m128i *)&index_data[i2], _mm_sub_epi32(r, r1));
  }
  for (i2 = scalarLB; i2 <= loop_ub; i2++) {
    index_data[i2] -= vk_data[i2] * b_varargin_1;
  }
  i2 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = b_index->size[1];
  emxEnsureCapacity_int32_T(&d_st, y, i2, &kd_emlrtRTEI);
  y_data = y->data;
  loop_ub = b_index->size[1];
  emxInit_int32_T(&d_st, &colIdx, 2, &sd_emlrtRTEI);
  i2 = colIdx->size[0] * colIdx->size[1];
  colIdx->size[0] = 1;
  colIdx->size[1] = vk->size[1];
  emxEnsureCapacity_int32_T(&d_st, colIdx, i2, &kd_emlrtRTEI);
  colIdx_data = colIdx->data;
  scalarLB = (b_index->size[1] / 4) << 2;
  vectorUB = scalarLB - 4;
  for (i2 = 0; i2 <= vectorUB; i2 += 4) {
    r = _mm_loadu_si128((const __m128i *)&index_data[i2]);
    r1 = _mm_set1_epi32(1);
    _mm_storeu_si128((__m128i *)&y_data[i2], _mm_add_epi32(r, r1));
    r = _mm_loadu_si128((const __m128i *)&vk_data[i2]);
    _mm_storeu_si128((__m128i *)&colIdx_data[i2], _mm_add_epi32(r, r1));
  }
  for (i2 = scalarLB; i2 < loop_ub; i2++) {
    y_data[i2] = index_data[i2] + 1;
    colIdx_data[i2] = vk_data[i2] + 1;
  }
  emxFree_int32_T(&d_st, &vk);
  emxInit_real_T(&c_st, &shiftedRowIdx, 2, &ld_emlrtRTEI);
  emxInit_real_T(&c_st, &shiftedColIdx, 2, &md_emlrtRTEI);
  emxInit_boolean_T(&c_st, &r2, 1, &rd_emlrtRTEI);
  emxInit_boolean_T(&c_st, &r3, 1, &rd_emlrtRTEI);
  emxInit_boolean_T(&c_st, &r4, 2, &wd_emlrtRTEI);
  emxInit_int32_T(&c_st, &r5, 1, &xd_emlrtRTEI);
  emxInit_boolean_T(&c_st, &r6, 1, &rd_emlrtRTEI);
  for (b_i = 0; b_i < 100; b_i++) {
    for (j = 0; j < 100; j++) {
      if (map[b_i + 100 * j]) {
        uint32_T varargin_2[2];
        boolean_T p;
        i2 = shiftedRowIdx->size[0] * shiftedRowIdx->size[1];
        shiftedRowIdx->size[0] = 1;
        loop_ub = y->size[1];
        shiftedRowIdx->size[1] = y->size[1];
        emxEnsureCapacity_real_T(&c_st, shiftedRowIdx, i2, &ld_emlrtRTEI);
        shiftedRowIdx_data = shiftedRowIdx->data;
        vectorUB = (b_i - i) + 1;
        i2 = shiftedColIdx->size[0] * shiftedColIdx->size[1];
        shiftedColIdx->size[0] = 1;
        shiftedColIdx->size[1] = colIdx->size[1];
        emxEnsureCapacity_real_T(&c_st, shiftedColIdx, i2, &md_emlrtRTEI);
        shiftedColIdx_data = shiftedColIdx->data;
        b_varargin_1 = (j - i1) + 1;
        for (i2 = 0; i2 < loop_ub; i2++) {
          shiftedRowIdx_data[i2] = (real_T)vectorUB + (real_T)y_data[i2];
          shiftedColIdx_data[i2] =
              (real_T)b_varargin_1 + (real_T)colIdx_data[i2];
        }
        i2 = r2->size[0];
        r2->size[0] = shiftedRowIdx->size[1];
        emxEnsureCapacity_boolean_T(&c_st, r2, i2, &nd_emlrtRTEI);
        r7 = r2->data;
        loop_ub = shiftedRowIdx->size[1];
        i2 = r3->size[0];
        r3->size[0] = shiftedColIdx->size[1];
        emxEnsureCapacity_boolean_T(&c_st, r3, i2, &od_emlrtRTEI);
        r8 = r3->data;
        for (i2 = 0; i2 < loop_ub; i2++) {
          r7[i2] = (shiftedRowIdx_data[i2] > 0.0);
          r8[i2] = (shiftedColIdx_data[i2] > 0.0);
        }
        if ((r2->size[0] != r3->size[0]) &&
            ((r2->size[0] != 1) && (r3->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(r2->size[0], r3->size[0], &c_emlrtECI,
                                      &c_st);
        }
        if (r2->size[0] == r3->size[0]) {
          loop_ub = r2->size[0];
          for (i2 = 0; i2 < loop_ub; i2++) {
            r7[i2] = (r7[i2] && r8[i2]);
          }
        } else {
          d_st.site = &tt_emlrtRSI;
          c_and(&d_st, r2, r3);
          r7 = r2->data;
        }
        i2 = r3->size[0];
        r3->size[0] = shiftedRowIdx->size[1];
        emxEnsureCapacity_boolean_T(&c_st, r3, i2, &pd_emlrtRTEI);
        r8 = r3->data;
        loop_ub = shiftedRowIdx->size[1];
        for (i2 = 0; i2 < loop_ub; i2++) {
          r8[i2] = (shiftedRowIdx_data[i2] <= 100.0);
        }
        if ((r2->size[0] != r3->size[0]) &&
            ((r2->size[0] != 1) && (r3->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(r2->size[0], r3->size[0], &c_emlrtECI,
                                      &c_st);
        }
        if (r2->size[0] == r3->size[0]) {
          loop_ub = r2->size[0];
          for (i2 = 0; i2 < loop_ub; i2++) {
            r7[i2] = (r7[i2] && r8[i2]);
          }
        } else {
          d_st.site = &tt_emlrtRSI;
          c_and(&d_st, r2, r3);
          r7 = r2->data;
        }
        i2 = r3->size[0];
        r3->size[0] = shiftedColIdx->size[1];
        emxEnsureCapacity_boolean_T(&c_st, r3, i2, &qd_emlrtRTEI);
        r8 = r3->data;
        loop_ub = shiftedColIdx->size[1];
        for (i2 = 0; i2 < loop_ub; i2++) {
          r8[i2] = (shiftedColIdx_data[i2] <= 100.0);
        }
        if ((r2->size[0] != r3->size[0]) &&
            ((r2->size[0] != 1) && (r3->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(r2->size[0], r3->size[0], &c_emlrtECI,
                                      &c_st);
        }
        if (r2->size[0] == r3->size[0]) {
          i2 = r6->size[0];
          r6->size[0] = r2->size[0];
          emxEnsureCapacity_boolean_T(&c_st, r6, i2, &rd_emlrtRTEI);
          r9 = r6->data;
          loop_ub = r2->size[0];
          for (i2 = 0; i2 < loop_ub; i2++) {
            r9[i2] = (r7[i2] && r8[i2]);
          }
        } else {
          d_st.site = &tt_emlrtRSI;
          b_and(&d_st, r6, r2, r3);
          r9 = r6->data;
        }
        scalarLB = r6->size[0] - 1;
        b_varargin_1 = 0;
        for (vectorUB = 0; vectorUB <= scalarLB; vectorUB++) {
          if (r9[vectorUB]) {
            b_varargin_1++;
          }
        }
        i2 = r5->size[0];
        r5->size[0] = b_varargin_1;
        emxEnsureCapacity_int32_T(&c_st, r5, i2, &sd_emlrtRTEI);
        vk_data = r5->data;
        b_varargin_1 = 0;
        for (vectorUB = 0; vectorUB <= scalarLB; vectorUB++) {
          if (r9[vectorUB]) {
            vk_data[b_varargin_1] = vectorUB;
            b_varargin_1++;
          }
        }
        d_st.site = &pc_emlrtRSI;
        loop_ub = r5->size[0];
        for (i2 = 0; i2 < loop_ub; i2++) {
          if ((vk_data[i2] < 0) || (vk_data[i2] > shiftedRowIdx->size[1] - 1)) {
            emlrtDynamicBoundsCheckR2012b(
                vk_data[i2], 0, shiftedRowIdx->size[1] - 1, &c_emlrtBCI, &d_st);
          }
        }
        loop_ub = r5->size[0];
        for (i2 = 0; i2 < loop_ub; i2++) {
          if ((vk_data[i2] < 0) || (vk_data[i2] > shiftedColIdx->size[1] - 1)) {
            emlrtDynamicBoundsCheckR2012b(
                vk_data[i2], 0, shiftedColIdx->size[1] - 1, &d_emlrtBCI, &d_st);
          }
        }
        e_st.site = &rc_emlrtRSI;
        scalarLB = 0;
        exitg1 = false;
        while ((!exitg1) && (scalarLB <= r5->size[0] - 1)) {
          radius = shiftedRowIdx_data[vk_data[scalarLB]];
          if ((radius >= 1.0) && (radius <= 100.0)) {
            scalarLB++;
          } else {
            emlrtErrorWithMessageIdR2018a(&e_st, &c_emlrtRTEI,
                                          "MATLAB:sub2ind:IndexOutOfRange",
                                          "MATLAB:sub2ind:IndexOutOfRange", 0);
          }
        }
        siz[0] = 1U;
        siz[1] = (uint32_T)r5->size[0];
        varargin_2[0] = 1U;
        varargin_2[1] = (uint32_T)r5->size[0];
        p = true;
        scalarLB = 0;
        exitg1 = false;
        while ((!exitg1) && (scalarLB < 2)) {
          if ((int32_T)siz[scalarLB] != (int32_T)varargin_2[scalarLB]) {
            p = false;
            exitg1 = true;
          } else {
            scalarLB++;
          }
        }
        if (!p) {
          emlrtErrorWithMessageIdR2018a(
              &e_st, &d_emlrtRTEI, "MATLAB:sub2ind:SubscriptVectorSize",
              "MATLAB:sub2ind:SubscriptVectorSize", 0);
        }
        scalarLB = 0;
        exitg1 = false;
        while ((!exitg1) && (scalarLB <= r5->size[0] - 1)) {
          radius = shiftedColIdx_data[vk_data[scalarLB]];
          if ((radius >= 1.0) && (radius <= 100.0)) {
            scalarLB++;
          } else {
            emlrtErrorWithMessageIdR2018a(&e_st, &c_emlrtRTEI,
                                          "MATLAB:sub2ind:IndexOutOfRange",
                                          "MATLAB:sub2ind:IndexOutOfRange", 0);
          }
        }
        i2 = b_index->size[0] * b_index->size[1];
        b_index->size[0] = 1;
        loop_ub = r5->size[0];
        b_index->size[1] = r5->size[0];
        emxEnsureCapacity_int32_T(&d_st, b_index, i2, &td_emlrtRTEI);
        index_data = b_index->data;
        for (i2 = 0; i2 < loop_ub; i2++) {
          index_data[i2] = (int32_T)shiftedRowIdx_data[vk_data[i2]] +
                           100 * ((int32_T)shiftedColIdx_data[vk_data[i2]] - 1);
        }
        b_varargin_1 = se->size[0] * se->size[1];
        loop_ub = r5->size[0];
        for (i2 = 0; i2 < loop_ub; i2++) {
          if ((vk_data[i2] < 0) || (vk_data[i2] > b_varargin_1 - 1)) {
            emlrtDynamicBoundsCheckR2012b(vk_data[i2], 0, b_varargin_1 - 1,
                                          &e_emlrtBCI, &c_st);
          }
        }
        loop_ub = b_index->size[1];
        for (i2 = 0; i2 < loop_ub; i2++) {
          b_varargin_1 = index_data[i2];
          if ((b_varargin_1 < 1) || (b_varargin_1 > 10000)) {
            emlrtDynamicBoundsCheckR2012b(b_varargin_1, 1, 10000, &f_emlrtBCI,
                                          &c_st);
          }
        }
        if ((r5->size[0] != b_index->size[1]) &&
            ((b_index->size[1] != 1) && (r5->size[0] != 1))) {
          emlrtDimSizeImpxCheckR2021b(b_index->size[1], r5->size[0],
                                      &b_emlrtECI, &c_st);
        }
        if (r5->size[0] == b_index->size[1]) {
          i2 = r4->size[0] * r4->size[1];
          r4->size[0] = 1;
          r4->size[1] = b_index->size[1];
          emxEnsureCapacity_boolean_T(&c_st, r4, i2, &ud_emlrtRTEI);
          r7 = r4->data;
          loop_ub = b_index->size[1];
          for (i2 = 0; i2 < loop_ub; i2++) {
            r7[i2] = (inflatedGrid[index_data[i2] - 1] || se_data[vk_data[i2]]);
          }
        } else {
          d_st.site = &yt_emlrtRSI;
          binary_expand_op(&d_st, r4, inflatedGrid, b_index, se, r5);
          r7 = r4->data;
        }
        if (b_index->size[1] != r4->size[1]) {
          emlrtSubAssignSizeCheck1dR2017a(b_index->size[1], r4->size[1],
                                          &emlrtECI, &c_st);
        }
        loop_ub = r4->size[1];
        for (i2 = 0; i2 < loop_ub; i2++) {
          inflatedGrid[index_data[i2] - 1] = r7[i2];
        }
      }
    }
  }
  emxFree_boolean_T(&c_st, &r6);
  emxFree_int32_T(&c_st, &y);
  emxFree_int32_T(&c_st, &r5);
  emxFree_boolean_T(&c_st, &r4);
  emxFree_boolean_T(&c_st, &r3);
  emxFree_boolean_T(&c_st, &r2);
  emxFree_int32_T(&c_st, &colIdx);
  emxFree_int32_T(&c_st, &b_index);
  emxFree_real_T(&c_st, &shiftedColIdx);
  emxFree_real_T(&c_st, &shiftedRowIdx);
  emxFree_boolean_T(&c_st, &se);
  st.site = &lb_emlrtRSI;
  memcpy(&obj->Buffer.Buffer[0], &inflatedGrid[0], 10000U * sizeof(boolean_T));
  obj->Index.Head[0] = 1.0;
  obj->Index.Head[1] = 1.0;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_binaryOccupancyMap_checkOccup(const emlrtStack *sp,
                                     const binaryOccupancyMap *obj,
                                     const emxArray_real_T *locs,
                                     emxArray_real_T *occupied)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_real_T *localGridInd;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &wg_emlrtRSI;
  emxInit_real_T(&st, &localGridInd, 2, &oh_emlrtRTEI);
  b_st.site = &xg_emlrtRSI;
  b_MapInterface_world2gridImpl(&b_st, obj, locs, localGridInd);
  b_st.site = &yg_emlrtRSI;
  c_MapLayer_getValueAtIndicesInt(&b_st, obj, localGridInd, occupied);
  emxFree_real_T(&st, &localGridInd);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (binaryOccupancyMap.c) */
