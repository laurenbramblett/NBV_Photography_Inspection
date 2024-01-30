/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * stateSpaceSE2.c
 *
 * Code generation for function 'stateSpaceSE2'
 *
 */

/* Include files */
#include "stateSpaceSE2.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "ixfun.h"
#include "repelem.h"
#include "rt_nonfinite.h"
#include "wrapToPi.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo bt_emlrtRSI =
    {
        335,                                /* lineNo */
        "stateSpaceSE2/enforceStateBounds", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pathName
                                                                          */
};

static emlrtRSInfo ct_emlrtRSI =
    {
        340,                                /* lineNo */
        "stateSpaceSE2/enforceStateBounds", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pathName
                                                                          */
};

static emlrtRSInfo dt_emlrtRSI =
    {
        341,                                /* lineNo */
        "stateSpaceSE2/enforceStateBounds", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pathName
                                                                          */
};

static emlrtRSInfo et_emlrtRSI =
    {
        344,                                /* lineNo */
        "stateSpaceSE2/enforceStateBounds", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pathName
                                                                          */
};

static emlrtRSInfo jt_emlrtRSI =
    {
        29,         /* lineNo */
        "minOrMax", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo kt_emlrtRSI =
    {
        58,         /* lineNo */
        "maximum2", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo lt_emlrtRSI = {
    63,               /* lineNo */
    "binaryMinOrMax", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
    "internal\\binaryMinOrMax.m" /* pathName */
};

static emlrtRSInfo mt_emlrtRSI = {
    57,      /* lineNo */
    "ixfun", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\ixfun.m" /* pathName
                                                                            */
};

static emlrtRSInfo ot_emlrtRSI =
    {
        31,         /* lineNo */
        "minOrMax", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtRSInfo pt_emlrtRSI =
    {
        67,         /* lineNo */
        "minimum2", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax."
        "m" /* pathName */
};

static emlrtECInfo ub_emlrtECI =
    {
        -1,                                 /* nDims */
        341,                                /* lineNo */
        13,                                 /* colNo */
        "stateSpaceSE2/enforceStateBounds", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pName
                                                                          */
};

static emlrtRTEInfo tk_emlrtRTEI =
    {
        341,             /* lineNo */
        33,              /* colNo */
        "stateSpaceSE2", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pName
                                                                          */
};

static emlrtRTEInfo uk_emlrtRTEI =
    {
        344,             /* lineNo */
        50,              /* colNo */
        "stateSpaceSE2", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pName
                                                                          */
};

static emlrtRTEInfo vk_emlrtRTEI = {
    15,    /* lineNo */
    5,     /* colNo */
    "max", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\datafun\\max.m" /* pName
                                                                        */
};

static emlrtRTEInfo wk_emlrtRTEI =
    {
        344,             /* lineNo */
        13,              /* colNo */
        "stateSpaceSE2", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pName
                                                                          */
};

static emlrtRTEInfo xk_emlrtRTEI =
    {
        344,             /* lineNo */
        85,              /* colNo */
        "stateSpaceSE2", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pName
                                                                          */
};

static emlrtRTEInfo yk_emlrtRTEI =
    {
        340,             /* lineNo */
        13,              /* colNo */
        "stateSpaceSE2", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pName
                                                                          */
};

static emlrtRTEInfo al_emlrtRTEI =
    {
        341,             /* lineNo */
        13,              /* colNo */
        "stateSpaceSE2", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pName
                                                                          */
};

static emlrtRTEInfo bl_emlrtRTEI =
    {
        344,             /* lineNo */
        32,              /* colNo */
        "stateSpaceSE2", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\nav\\navalgs2\\stateSpaceSE2.m" /* pName
                                                                          */
};

/* Function Definitions */
void c_stateSpaceSE2_enforceStateBou(const emlrtStack *sp,
                                     const stateSpaceSE2 *obj,
                                     emxArray_real_T *state)
{
  static real_T dv[2] = {0.0, 3.0};
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack st;
  emxArray_real_T *b_bounds;
  emxArray_real_T *bounds;
  emxArray_real_T *maxval;
  emxArray_real_T *r;
  real_T varargin_1;
  real_T varargin_2;
  real_T *b_bounds_data;
  real_T *bounds_data;
  real_T *state_data;
  int32_T i;
  int32_T i1;
  int32_T k;
  boolean_T p;
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
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  dv[0U] = rtNaN;
  state_data = state->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  st.site = &bt_emlrtRSI;
  b_st.site = &bo_emlrtRSI;
  c_st.site = &ab_emlrtRSI;
  if (state->size[0] == 0) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &pb_emlrtRTEI,
        "Coder:toolbox:ValidateattributesexpectedNonempty",
        "MATLAB:enforceStateBounds:expectedNonempty", 3, 4, 5, "state");
  }
  c_st.site = &ab_emlrtRSI;
  p = true;
  for (k = 0; k < 2; k++) {
    if (p) {
      varargin_1 = dv[k];
      if ((!(varargin_1 != varargin_1)) && (state->size[k] != 3)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    emlrtErrorWithMessageIdR2018a(
        &c_st, &tb_emlrtRTEI, "Coder:toolbox:ValidateattributesincorrectSize",
        "MATLAB:enforceStateBounds:incorrectSize", 3, 4, 5, "state");
  }
  emxInit_real_T(sp, &bounds, 2, &yk_emlrtRTEI);
  st.site = &ct_emlrtRSI;
  repelem(&st, obj->StateBoundsInternal, state->size[0], bounds);
  bounds_data = bounds->data;
  emxInit_real_T(sp, &r, 1, &al_emlrtRTEI);
  i = r->size[0];
  r->size[0] = state->size[0];
  emxEnsureCapacity_real_T(sp, r, i, &tk_emlrtRTEI);
  b_bounds_data = r->data;
  k = state->size[0];
  for (i = 0; i < k; i++) {
    b_bounds_data[i] = state_data[i + state->size[0] * 2];
  }
  st.site = &dt_emlrtRSI;
  b_wrapToPi(&st, r);
  b_bounds_data = r->data;
  emlrtSubAssignSizeCheckR2012b(&state->size[0], 1, &r->size[0], 1,
                                &ub_emlrtECI, (emlrtCTX)sp);
  k = r->size[0];
  for (i = 0; i < k; i++) {
    state_data[i + state->size[0] * 2] = b_bounds_data[i];
  }
  emxFree_real_T(sp, &r);
  emxInit_real_T(sp, &maxval, 2, &bl_emlrtRTEI);
  st.site = &et_emlrtRSI;
  b_st.site = &ns_emlrtRSI;
  c_st.site = &jt_emlrtRSI;
  d_st.site = &kt_emlrtRSI;
  e_st.site = &lt_emlrtRSI;
  f_st.site = &mt_emlrtRSI;
  emxInit_real_T(&f_st, &b_bounds, 2, &uk_emlrtRTEI);
  if (state->size[0] == bounds->size[0]) {
    i = maxval->size[0] * maxval->size[1];
    maxval->size[0] = state->size[0];
    maxval->size[1] = 3;
    emxEnsureCapacity_real_T(&f_st, maxval, i, &vk_emlrtRTEI);
    b_bounds_data = maxval->data;
    k = state->size[0];
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < k; i1++) {
        varargin_1 = state_data[i1 + state->size[0] * i];
        varargin_2 = bounds_data[i1 + bounds->size[0] * i];
        b_bounds_data[i1 + maxval->size[0] * i] =
            muDoubleScalarMax(varargin_1, varargin_2);
      }
    }
  } else {
    i = b_bounds->size[0] * b_bounds->size[1];
    b_bounds->size[0] = bounds->size[0];
    b_bounds->size[1] = 3;
    emxEnsureCapacity_real_T(&f_st, b_bounds, i, &uk_emlrtRTEI);
    b_bounds_data = b_bounds->data;
    k = bounds->size[0];
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < k; i1++) {
        b_bounds_data[i1 + b_bounds->size[0] * i] =
            bounds_data[i1 + bounds->size[0] * i];
      }
    }
    g_st.site = &nt_emlrtRSI;
    expand_max(&g_st, state, b_bounds, maxval);
    b_bounds_data = maxval->data;
  }
  st.site = &et_emlrtRSI;
  b_st.site = &hj_emlrtRSI;
  c_st.site = &ot_emlrtRSI;
  d_st.site = &pt_emlrtRSI;
  e_st.site = &lt_emlrtRSI;
  f_st.site = &mt_emlrtRSI;
  if (maxval->size[0] == bounds->size[0]) {
    i = state->size[0] * state->size[1];
    state->size[0] = maxval->size[0];
    state->size[1] = 3;
    emxEnsureCapacity_real_T(&f_st, state, i, &wk_emlrtRTEI);
    state_data = state->data;
    k = maxval->size[0];
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < k; i1++) {
        varargin_1 = b_bounds_data[i1 + maxval->size[0] * i];
        varargin_2 = bounds_data[i1 + bounds->size[0] * (i + 3)];
        state_data[i1 + state->size[0] * i] =
            muDoubleScalarMin(varargin_1, varargin_2);
      }
    }
  } else {
    i = b_bounds->size[0] * b_bounds->size[1];
    b_bounds->size[0] = bounds->size[0];
    b_bounds->size[1] = 3;
    emxEnsureCapacity_real_T(&f_st, b_bounds, i, &xk_emlrtRTEI);
    b_bounds_data = b_bounds->data;
    k = bounds->size[0];
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < k; i1++) {
        b_bounds_data[i1 + b_bounds->size[0] * i] =
            bounds_data[i1 + bounds->size[0] * (i + 3)];
      }
    }
    g_st.site = &nt_emlrtRSI;
    expand_min(&g_st, maxval, b_bounds, state);
  }
  emxFree_real_T(&f_st, &b_bounds);
  emxFree_real_T(&f_st, &maxval);
  emxFree_real_T(&f_st, &bounds);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (stateSpaceSE2.c) */
