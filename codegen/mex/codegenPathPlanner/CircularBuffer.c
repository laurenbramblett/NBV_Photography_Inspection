/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * CircularBuffer.c
 *
 * Code generation for function 'CircularBuffer'
 *
 */

/* Include files */
#include "CircularBuffer.h"
#include "CircularBufferIndex.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo dh_emlrtRSI = {
    78,                                 /* lineNo */
    "CircularBuffer/getValueAtIndices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m" /* pathName */
};

static emlrtRSInfo eh_emlrtRSI = {
    79,                                 /* lineNo */
    "CircularBuffer/getValueAtIndices", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m" /* pathName */
};

static emlrtECInfo i_emlrtECI = {
    -1,                                           /* nDims */
    215,                                          /* lineNo */
    25,                                           /* colNo */
    "CircularBuffer/getBaseMatrixValueAtIndices", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m" /* pName */
};

static emlrtBCInfo t_emlrtBCI = {
    1,                                            /* iFirst */
    10000,                                        /* iLast */
    218,                                          /* lineNo */
    37,                                           /* colNo */
    "",                                           /* aName */
    "CircularBuffer/getBaseMatrixValueAtIndices", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m", /* pName */
    0            /* checkKind */
};

static emlrtDCInfo g_emlrtDCI = {
    218,                                          /* lineNo */
    37,                                           /* colNo */
    "CircularBuffer/getBaseMatrixValueAtIndices", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m", /* pName */
    1            /* checkKind */
};

static emlrtRTEInfo re_emlrtRTEI = {
    78,               /* lineNo */
    13,               /* colNo */
    "CircularBuffer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m" /* pName */
};

static emlrtRTEInfo we_emlrtRTEI = {
    215,              /* lineNo */
    25,               /* colNo */
    "CircularBuffer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m" /* pName */
};

static emlrtRTEInfo xe_emlrtRTEI = {
    218,              /* lineNo */
    17,               /* colNo */
    "CircularBuffer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\Circula"
    "rBuffer.m" /* pName */
};

/* Function Declarations */
static void c_CircularBuffer_getBaseMatrixV(
    const emlrtStack *sp, const e_matlabshared_autonomous_inter *obj,
    const emxArray_real_T *indices, emxArray_boolean_T *values);

/* Function Definitions */
static void c_CircularBuffer_getBaseMatrixV(
    const emlrtStack *sp, const e_matlabshared_autonomous_inter *obj,
    const emxArray_real_T *indices, emxArray_boolean_T *values)
{
  emxArray_real_T *r;
  const real_T *indices_data;
  real_T *r1;
  int32_T i;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  boolean_T *values_data;
  indices_data = indices->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &r, 1, &we_emlrtRTEI);
  i = r->size[0];
  r->size[0] = indices->size[0];
  emxEnsureCapacity_real_T(sp, r, i, &we_emlrtRTEI);
  r1 = r->data;
  loop_ub = indices->size[0];
  scalarLB = (indices->size[0] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i = 0; i <= vectorUB; i += 2) {
    _mm_storeu_pd(
        &r1[i],
        _mm_mul_pd(_mm_set1_pd(100.0),
                   _mm_sub_pd(_mm_loadu_pd(&indices_data[i + indices->size[0]]),
                              _mm_set1_pd(1.0))));
  }
  for (i = scalarLB; i < loop_ub; i++) {
    r1[i] = 100.0 * (indices_data[i + indices->size[0]] - 1.0);
  }
  if (r->size[0] != indices->size[0]) {
    emlrtSizeEqCheck1DR2012b(r->size[0], indices->size[0], &i_emlrtECI,
                             (emlrtConstCTX)sp);
  }
  i = values->size[0];
  values->size[0] = r->size[0];
  emxEnsureCapacity_boolean_T(sp, values, i, &xe_emlrtRTEI);
  values_data = values->data;
  loop_ub = r->size[0];
  for (i = 0; i < loop_ub; i++) {
    real_T d;
    d = r1[i] + indices_data[i];
    if (d != (int32_T)muDoubleScalarFloor(d)) {
      emlrtIntegerCheckR2012b(d, &g_emlrtDCI, (emlrtConstCTX)sp);
    }
    if (((int32_T)d < 1) || ((int32_T)d > 10000)) {
      emlrtDynamicBoundsCheckR2012b((int32_T)d, 1, 10000, &t_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    values_data[i] = obj->Buffer[(int32_T)d - 1];
  }
  emxFree_real_T(sp, &r);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void c_CircularBuffer_getValueAtIndi(const emlrtStack *sp,
                                     const e_matlabshared_autonomous_inter *obj,
                                     const emxArray_real_T *indices,
                                     emxArray_boolean_T *values)
{
  emlrtStack st;
  emxArray_real_T *ind;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &ind, 2, &re_emlrtRTEI);
  st.site = &dh_emlrtRSI;
  c_CircularBufferIndex_toBaseMat(&st, obj->Index, indices, ind);
  st.site = &eh_emlrtRSI;
  c_CircularBuffer_getBaseMatrixV(&st, obj, ind, values);
  emxFree_real_T(sp, &ind);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (CircularBuffer.c) */
