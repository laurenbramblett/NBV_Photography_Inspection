/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * wrapToPi.c
 *
 * Code generation for function 'wrapToPi'
 *
 */

/* Include files */
#include "wrapToPi.h"
#include "allOrAny.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "wrapTo2Pi.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ht_emlrtRSI = {
    14,         /* lineNo */
    "wrapToPi", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\robotics\\robotutils\\+robotics\\+"
    "internal\\wrapToPi.m" /* pathName */
};

static emlrtRSInfo it_emlrtRSI = {
    18,         /* lineNo */
    "wrapToPi", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\robotics\\robotutils\\+robotics\\+"
    "internal\\wrapToPi.m" /* pathName */
};

static emlrtECInfo vb_emlrtECI = {
    1,           /* nDims */
    23,          /* lineNo */
    11,          /* colNo */
    "wrapTo2Pi", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\robotics\\robotutils\\+robotics\\+"
    "internal\\wrapTo2Pi.m" /* pName */
};

static emlrtBCInfo mg_emlrtBCI = {
    -1,          /* iFirst */
    -1,          /* iLast */
    23,          /* lineNo */
    11,          /* colNo */
    "",          /* aName */
    "wrapTo2Pi", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\robotics\\robotutils\\+robotics\\+"
    "internal\\wrapTo2Pi.m", /* pName */
    0                        /* checkKind */
};

static emlrtRTEInfo cl_emlrtRTEI = {
    14,         /* lineNo */
    12,         /* colNo */
    "wrapToPi", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\robotics\\robotutils\\+robotics\\+"
    "internal\\wrapToPi.m" /* pName */
};

static emlrtRTEInfo dl_emlrtRTEI = {
    18,         /* lineNo */
    45,         /* colNo */
    "wrapToPi", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\robotics\\robotutils\\+robotics\\+"
    "internal\\wrapToPi.m" /* pName */
};

static emlrtRTEInfo el_emlrtRTEI = {
    20,          /* lineNo */
    1,           /* colNo */
    "wrapTo2Pi", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\robotics\\robotutils\\+robotics\\+"
    "internal\\wrapTo2Pi.m" /* pName */
};

/* Function Definitions */
void b_wrapToPi(const emlrtStack *sp, emxArray_real_T *theta)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  emxArray_boolean_T b_pos_data;
  emxArray_real_T *y;
  real_T *theta_data;
  real_T *y_data;
  int32_T i;
  int32_T k;
  int32_T loop_ub;
  int32_T pos_size;
  int32_T vectorUB;
  boolean_T b_tmp_data[50000];
  boolean_T pos_data[50000];
  boolean_T b_y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  theta_data = theta->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &ht_emlrtRSI;
  emxInit_real_T(&st, &y, 1, &cl_emlrtRTEI);
  b_st.site = &ht_emlrtRSI;
  c_st.site = &mo_emlrtRSI;
  i = theta->size[0];
  k = y->size[0];
  y->size[0] = theta->size[0];
  emxEnsureCapacity_real_T(&c_st, y, k, &eh_emlrtRTEI);
  y_data = y->data;
  for (k = 0; k < i; k++) {
    y_data[k] = muDoubleScalarAbs(theta_data[k]);
  }
  b_st.site = &oo_emlrtRSI;
  pos_size = y->size[0];
  loop_ub = y->size[0];
  for (i = 0; i < loop_ub; i++) {
    pos_data[i] = (y_data[i] > 3.1415926535897931);
  }
  b_pos_data.data = &pos_data[0];
  b_pos_data.size = &pos_size;
  b_pos_data.allocatedSize = 50000;
  b_pos_data.numDimensions = 1;
  b_pos_data.canFreeData = false;
  b_y = allOrAny_anonFcn1(&b_pos_data);
  if (b_y) {
    __m128d r;
    boolean_T tmp_data[50000];
    st.site = &it_emlrtRSI;
    i = y->size[0];
    y->size[0] = theta->size[0];
    emxEnsureCapacity_real_T(&st, y, i, &dl_emlrtRTEI);
    y_data = y->data;
    loop_ub = theta->size[0];
    k = (loop_ub / 2) << 1;
    vectorUB = k - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&theta_data[i]);
      _mm_storeu_pd(&y_data[i], _mm_add_pd(r, _mm_set1_pd(3.1415926535897931)));
    }
    for (i = k; i < loop_ub; i++) {
      y_data[i] = theta_data[i] + 3.1415926535897931;
    }
    pos_size = y->size[0];
    loop_ub = y->size[0];
    i = theta->size[0];
    theta->size[0] = y->size[0];
    emxEnsureCapacity_real_T(&st, theta, i, &el_emlrtRTEI);
    theta_data = theta->data;
    for (i = 0; i < loop_ub; i++) {
      real_T varargin_1;
      pos_data[i] = (y_data[i] > 0.0);
      varargin_1 = y_data[i];
      theta_data[i] = b_mod(varargin_1, 6.2831853071795862);
    }
    k = theta->size[0];
    loop_ub = theta->size[0];
    for (i = 0; i < loop_ub; i++) {
      tmp_data[i] = (theta_data[i] == 0.0);
    }
    if ((k != pos_size) && ((k != 1) && (pos_size != 1))) {
      emlrtDimSizeImpxCheckR2021b(k, pos_size, &vb_emlrtECI, &st);
    }
    if (k == pos_size) {
      for (i = 0; i < k; i++) {
        b_tmp_data[i] = (tmp_data[i] && pos_data[i]);
      }
    } else {
      k = e_and(b_tmp_data, tmp_data, &k, pos_data, &pos_size);
    }
    k--;
    for (vectorUB = 0; vectorUB <= k; vectorUB++) {
      if (b_tmp_data[vectorUB]) {
        i = theta->size[0] - 1;
        if (vectorUB > i) {
          emlrtDynamicBoundsCheckR2012b(vectorUB, 0, i, &mg_emlrtBCI, &st);
        }
        theta_data[vectorUB] = 6.2831853071795862;
      }
    }
    loop_ub = theta->size[0];
    k = (loop_ub / 2) << 1;
    vectorUB = k - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&theta_data[i]);
      _mm_storeu_pd(&theta_data[i],
                    _mm_sub_pd(r, _mm_set1_pd(3.1415926535897931)));
    }
    for (i = k; i < loop_ub; i++) {
      theta_data[i] -= 3.1415926535897931;
    }
  }
  emxFree_real_T(sp, &y);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

void wrapToPi(real_T *theta)
{
  if (muDoubleScalarAbs(*theta) > 3.1415926535897931) {
    real_T thetaWrap;
    if (muDoubleScalarIsNaN(*theta + 3.1415926535897931) ||
        muDoubleScalarIsInf(*theta + 3.1415926535897931)) {
      thetaWrap = rtNaN;
    } else if (*theta + 3.1415926535897931 == 0.0) {
      thetaWrap = 0.0;
    } else {
      boolean_T rEQ0;
      thetaWrap =
          muDoubleScalarRem(*theta + 3.1415926535897931, 6.2831853071795862);
      rEQ0 = (thetaWrap == 0.0);
      if (!rEQ0) {
        real_T q;
        q = muDoubleScalarAbs((*theta + 3.1415926535897931) /
                              6.2831853071795862);
        rEQ0 = !(muDoubleScalarAbs(q - muDoubleScalarFloor(q + 0.5)) >
                 2.2204460492503131E-16 * q);
      }
      if (rEQ0) {
        thetaWrap = 0.0;
      } else if (*theta + 3.1415926535897931 < 0.0) {
        thetaWrap += 6.2831853071795862;
      }
    }
    if ((thetaWrap == 0.0) && (*theta + 3.1415926535897931 > 0.0)) {
      thetaWrap = 6.2831853071795862;
    }
    *theta = thetaWrap - 3.1415926535897931;
  }
}

/* End of code generation (wrapToPi.c) */
