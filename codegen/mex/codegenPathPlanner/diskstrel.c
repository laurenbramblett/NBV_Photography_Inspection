/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diskstrel.c
 *
 * Code generation for function 'diskstrel'
 *
 */

/* Include files */
#include "diskstrel.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "eml_int_forloop_overflow_check.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo dc_emlrtRSI = {
    15,          /* lineNo */
    "diskstrel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pathName */
};

static emlrtRSInfo ec_emlrtRSI = {
    16,          /* lineNo */
    "diskstrel", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pathName */
};

static emlrtRSInfo fc_emlrtRSI =
    {
        28,      /* lineNo */
        "colon", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pathName
                                                                          */
};

static emlrtRSInfo gc_emlrtRSI =
    {
        125,     /* lineNo */
        "colon", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pathName
                                                                          */
};

static emlrtRSInfo hc_emlrtRSI =
    {
        319,               /* lineNo */
        "eml_float_colon", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pathName
                                                                          */
};

static emlrtRSInfo jc_emlrtRSI = {
    31,         /* lineNo */
    "meshgrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\meshgrid.m" /* pathName
                                                                           */
};

static emlrtRSInfo kc_emlrtRSI = {
    32,         /* lineNo */
    "meshgrid", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\meshgrid.m" /* pathName
                                                                           */
};

static emlrtRSInfo lc_emlrtRSI =
    {
        71,      /* lineNo */
        "power", /* fcnName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\power.m" /* pathName
                                                                          */
};

static emlrtRSInfo mc_emlrtRSI = {
    44,       /* lineNo */
    "mpower", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\matfun\\mpower.m" /* pathName
                                                                          */
};

static emlrtRTEInfo k_emlrtRTEI =
    {
        419,               /* lineNo */
        15,                /* colNo */
        "assert_pmaxsize", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pName
                                                                          */
};

static emlrtECInfo d_emlrtECI = {
    2,           /* nDims */
    16,          /* lineNo */
    10,          /* colNo */
    "diskstrel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pName */
};

static emlrtECInfo e_emlrtECI = {
    1,           /* nDims */
    16,          /* lineNo */
    10,          /* colNo */
    "diskstrel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pName */
};

static emlrtRTEInfo ae_emlrtRTEI = {
    20,         /* lineNo */
    25,         /* colNo */
    "meshgrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\meshgrid.m" /* pName
                                                                           */
};

static emlrtRTEInfo be_emlrtRTEI = {
    21,         /* lineNo */
    25,         /* colNo */
    "meshgrid", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\meshgrid.m" /* pName
                                                                           */
};

static emlrtRTEInfo ce_emlrtRTEI =
    {
        320,     /* lineNo */
        20,      /* colNo */
        "colon", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m" /* pName
                                                                          */
};

static emlrtRTEInfo de_emlrtRTEI = {
    16,          /* lineNo */
    10,          /* colNo */
    "diskstrel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pName */
};

static emlrtRTEInfo ee_emlrtRTEI = {
    16,          /* lineNo */
    17,          /* colNo */
    "diskstrel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pName */
};

static emlrtRTEInfo fe_emlrtRTEI = {
    16,          /* lineNo */
    5,           /* colNo */
    "diskstrel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pName */
};

static emlrtRTEInfo ge_emlrtRTEI = {
    1,           /* lineNo */
    15,          /* colNo */
    "diskstrel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pName */
};

static emlrtRTEInfo he_emlrtRTEI = {
    15,          /* lineNo */
    22,          /* colNo */
    "diskstrel", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\nav_rst\\nav_rst_lib\\+nav\\+"
    "algs\\+internal\\diskstrel.m" /* pName */
};

/* Function Declarations */
static void b_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real_T *in3, real_T in4);

/* Function Definitions */
static void b_binary_expand_op(const emlrtStack *sp, emxArray_boolean_T *in1,
                               const emxArray_real_T *in2,
                               const emxArray_real_T *in3, real_T in4)
{
  const real_T *in2_data;
  const real_T *in3_data;
  int32_T aux_0_1;
  int32_T aux_1_1;
  int32_T b_loop_ub;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_0_1;
  int32_T stride_1_0;
  int32_T stride_1_1;
  boolean_T *in1_data;
  in3_data = in3->data;
  in2_data = in2->data;
  if (in3->size[0] == 1) {
    loop_ub = in2->size[0];
  } else {
    loop_ub = in3->size[0];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, in1, i, &fe_emlrtRTEI);
  if (in3->size[1] == 1) {
    b_loop_ub = in2->size[1];
  } else {
    b_loop_ub = in3->size[1];
  }
  i = in1->size[0] * in1->size[1];
  in1->size[1] = b_loop_ub;
  emxEnsureCapacity_boolean_T(sp, in1, i, &fe_emlrtRTEI);
  in1_data = in1->data;
  stride_0_0 = (in2->size[0] != 1);
  stride_0_1 = (in2->size[1] != 1);
  stride_1_0 = (in3->size[0] != 1);
  stride_1_1 = (in3->size[1] != 1);
  aux_0_1 = 0;
  aux_1_1 = 0;
  for (i = 0; i < b_loop_ub; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      in1_data[i1 + in1->size[0] * i] =
          (in2_data[i1 * stride_0_0 + in2->size[0] * aux_0_1] +
               in3_data[i1 * stride_1_0 + in3->size[0] * aux_1_1] <=
           in4);
    }
    aux_1_1 += stride_1_1;
    aux_0_1 += stride_0_1;
  }
}

void diskstrel(const emlrtStack *sp, real_T r, emxArray_boolean_T *se)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack st;
  emxArray_real_T *b_r;
  emxArray_real_T *b_y;
  emxArray_real_T *x;
  emxArray_real_T *y;
  real_T ndbl;
  real_T *b_y_data;
  real_T *x_data;
  real_T *y_data;
  int32_T k;
  int32_T n;
  int32_T nm1d2;
  boolean_T *se_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_real_T(sp, &y, 2, &he_emlrtRTEI);
  y_data = y->data;
  st.site = &dc_emlrtRSI;
  b_st.site = &fc_emlrtRSI;
  if (muDoubleScalarIsNaN(-r) || muDoubleScalarIsNaN(r)) {
    k = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity_real_T(&b_st, y, k, &fd_emlrtRTEI);
    y_data = y->data;
    y_data[0] = rtNaN;
  } else if (r < -r) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else if ((muDoubleScalarIsInf(-r) || muDoubleScalarIsInf(r)) && (-r == r)) {
    k = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity_real_T(&b_st, y, k, &fd_emlrtRTEI);
    y_data = y->data;
    y_data[0] = rtNaN;
  } else if (muDoubleScalarFloor(-r) == -r) {
    k = y->size[0] * y->size[1];
    y->size[0] = 1;
    nm1d2 = (int32_T)(r - (-r));
    y->size[1] = nm1d2 + 1;
    emxEnsureCapacity_real_T(&b_st, y, k, &fd_emlrtRTEI);
    y_data = y->data;
    for (k = 0; k <= nm1d2; k++) {
      y_data[k] = -r + (real_T)k;
    }
  } else {
    real_T apnd;
    real_T cdiff;
    c_st.site = &gc_emlrtRSI;
    ndbl = muDoubleScalarFloor((r - (-r)) + 0.5);
    apnd = -r + ndbl;
    cdiff = apnd - r;
    if (muDoubleScalarAbs(cdiff) <
        4.4408920985006262E-16 *
            muDoubleScalarMax(muDoubleScalarAbs(-r), muDoubleScalarAbs(r))) {
      ndbl++;
      apnd = r;
    } else if (cdiff > 0.0) {
      apnd = -r + (ndbl - 1.0);
    } else {
      ndbl++;
    }
    if (ndbl >= 0.0) {
      n = (int32_T)ndbl;
    } else {
      n = 0;
    }
    d_st.site = &hc_emlrtRSI;
    if (ndbl > 2.147483647E+9) {
      emlrtErrorWithMessageIdR2018a(&d_st, &k_emlrtRTEI,
                                    "Coder:MATLAB:pmaxsize",
                                    "Coder:MATLAB:pmaxsize", 0);
    }
    k = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = n;
    emxEnsureCapacity_real_T(&c_st, y, k, &ce_emlrtRTEI);
    y_data = y->data;
    if (n > 0) {
      y_data[0] = -r;
      if (n > 1) {
        y_data[n - 1] = apnd;
        nm1d2 = (n - 1) / 2;
        for (k = 0; k <= nm1d2 - 2; k++) {
          y_data[k + 1] = -r + ((real_T)k + 1.0);
          y_data[(n - k) - 2] = apnd - ((real_T)k + 1.0);
        }
        if (nm1d2 << 1 == n - 1) {
          y_data[nm1d2] = (-r + apnd) / 2.0;
        } else {
          y_data[nm1d2] = -r + (real_T)nm1d2;
          y_data[nm1d2 + 1] = apnd - (real_T)nm1d2;
        }
      }
    }
  }
  st.site = &dc_emlrtRSI;
  nm1d2 = y->size[1];
  emxInit_real_T(&st, &x, 2, &de_emlrtRTEI);
  k = x->size[0] * x->size[1];
  x->size[0] = y->size[1];
  x->size[1] = y->size[1];
  emxEnsureCapacity_real_T(&st, x, k, &ae_emlrtRTEI);
  x_data = x->data;
  emxInit_real_T(&st, &b_y, 2, &ge_emlrtRTEI);
  k = b_y->size[0] * b_y->size[1];
  b_y->size[0] = y->size[1];
  b_y->size[1] = y->size[1];
  emxEnsureCapacity_real_T(&st, b_y, k, &be_emlrtRTEI);
  b_y_data = b_y->data;
  if (y->size[1] != 0) {
    b_st.site = &jc_emlrtRSI;
    if (y->size[1] > 2147483646) {
      c_st.site = &wb_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }
    for (n = 0; n < nm1d2; n++) {
      b_st.site = &kc_emlrtRSI;
      if (nm1d2 > 2147483646) {
        c_st.site = &wb_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }
      for (k = 0; k < nm1d2; k++) {
        x_data[k + x->size[0] * n] = y_data[n];
        b_y_data[k + b_y->size[0] * n] = y_data[k];
      }
    }
  }
  emxFree_real_T(&st, &y);
  st.site = &ec_emlrtRSI;
  b_st.site = &lc_emlrtRSI;
  emxInit_real_T(&b_st, &b_r, 2, &de_emlrtRTEI);
  k = b_r->size[0] * b_r->size[1];
  b_r->size[0] = x->size[0];
  b_r->size[1] = x->size[1];
  emxEnsureCapacity_real_T(&b_st, b_r, k, &de_emlrtRTEI);
  y_data = b_r->data;
  nm1d2 = x->size[0] * x->size[1];
  for (k = 0; k < nm1d2; k++) {
    ndbl = x_data[k];
    y_data[k] = ndbl * ndbl;
  }
  st.site = &ec_emlrtRSI;
  b_st.site = &lc_emlrtRSI;
  k = x->size[0] * x->size[1];
  x->size[0] = b_y->size[0];
  x->size[1] = b_y->size[1];
  emxEnsureCapacity_real_T(&b_st, x, k, &ee_emlrtRTEI);
  x_data = x->data;
  for (k = 0; k < nm1d2; k++) {
    ndbl = b_y_data[k];
    x_data[k] = ndbl * ndbl;
  }
  emxFree_real_T(&b_st, &b_y);
  if ((b_r->size[0] != x->size[0]) &&
      ((b_r->size[0] != 1) && (x->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_r->size[0], x->size[0], &e_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if ((b_r->size[1] != x->size[1]) &&
      ((b_r->size[1] != 1) && (x->size[1] != 1))) {
    emlrtDimSizeImpxCheckR2021b(b_r->size[1], x->size[1], &d_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  st.site = &ec_emlrtRSI;
  b_st.site = &mc_emlrtRSI;
  c_st.site = &lc_emlrtRSI;
  ndbl = (r + 0.75) * (r + 0.75);
  if ((b_r->size[0] == x->size[0]) && (b_r->size[1] == x->size[1])) {
    k = se->size[0] * se->size[1];
    se->size[0] = b_r->size[0];
    se->size[1] = b_r->size[1];
    emxEnsureCapacity_boolean_T(sp, se, k, &fe_emlrtRTEI);
    se_data = se->data;
    for (k = 0; k < nm1d2; k++) {
      se_data[k] = (y_data[k] + x_data[k] <= ndbl);
    }
  } else {
    st.site = &ec_emlrtRSI;
    b_binary_expand_op(&st, se, b_r, x, ndbl);
  }
  emxFree_real_T(sp, &b_r);
  emxFree_real_T(sp, &x);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (diskstrel.c) */
