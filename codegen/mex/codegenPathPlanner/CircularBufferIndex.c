/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * CircularBufferIndex.c
 *
 * Code generation for function 'CircularBufferIndex'
 *
 */

/* Include files */
#include "CircularBufferIndex.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "ixfun.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRTEInfo se_emlrtRTEI = {
    138,                   /* lineNo */
    23,                    /* colNo */
    "CircularBufferIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBufferIndex.m" /* pName */
};

static emlrtRTEInfo te_emlrtRTEI =
    {
        75,    /* lineNo */
        5,     /* colNo */
        "mod", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elfun\\mod.m" /* pName
                                                                          */
};

static emlrtRTEInfo ue_emlrtRTEI = {
    80,                    /* lineNo */
    13,                    /* colNo */
    "CircularBufferIndex", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBufferIndex.m" /* pName */
};

/* Function Definitions */
void c_CircularBufferIndex_toBaseMat(const emlrtStack *sp,
                                     const d_matlabshared_autonomous_inter *obj,
                                     const emxArray_real_T *b_index,
                                     emxArray_real_T *region)
{
  __m128d r;
  emlrtStack st;
  emxArray_real_T *x;
  const real_T *index_data;
  real_T *region_data;
  real_T *x_data;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  st.prev = sp;
  st.tls = sp->tls;
  index_data = b_index->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &x, 2, &se_emlrtRTEI);
  i = x->size[0] * x->size[1];
  x->size[0] = b_index->size[0];
  x->size[1] = 2;
  emxEnsureCapacity_real_T(sp, x, i, &se_emlrtRTEI);
  x_data = x->data;
  loop_ub = b_index->size[0];
  scalarLB = (loop_ub / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i < 2; i++) {
    for (i1 = 0; i1 <= vectorUB; i1 += 2) {
      r = _mm_set1_pd(1.0);
      _mm_storeu_pd(
          &x_data[i1 + x->size[0] * i],
          _mm_sub_pd(
              _mm_sub_pd(
                  _mm_add_pd(
                      _mm_set1_pd(obj->Head[i]),
                      _mm_loadu_pd(&index_data[i1 + b_index->size[0] * i])),
                  r),
              r));
    }
    for (i1 = scalarLB; i1 < loop_ub; i1++) {
      x_data[i1 + x->size[0] * i] =
          ((obj->Head[i] + index_data[i1 + b_index->size[0] * i]) - 1.0) - 1.0;
    }
  }
  if (x->size[0] == 1) {
    real_T varargin_1;
    i = region->size[0] * region->size[1];
    region->size[0] = 1;
    region->size[1] = 2;
    emxEnsureCapacity_real_T(sp, region, i, &te_emlrtRTEI);
    region_data = region->data;
    varargin_1 = x_data[0];
    region_data[0] = b_mod(varargin_1, 100.0);
    varargin_1 = x_data[1];
    region_data[1] = b_mod(varargin_1, 100.0);
  } else {
    st.site = &nt_emlrtRSI;
    expand_mod(&st, x, region);
  }
  emxFree_real_T(sp, &x);
  loop_ub = region->size[0] << 1;
  i = region->size[0] * region->size[1];
  region->size[1] = 2;
  emxEnsureCapacity_real_T(sp, region, i, &ue_emlrtRTEI);
  region_data = region->data;
  scalarLB = (loop_ub / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    r = _mm_loadu_pd(&region_data[i]);
    _mm_storeu_pd(&region_data[i], _mm_add_pd(r, _mm_set1_pd(1.0)));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    region_data[i]++;
  }
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (CircularBufferIndex.c) */
