/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * MapLayer.c
 *
 * Code generation for function 'MapLayer'
 *
 */

/* Include files */
#include "MapLayer.h"
#include "CircularBuffer.h"
#include "any.h"
#include "codegenPathPlanner_data.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "eml_int_forloop_overflow_check.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "validatorOccupancyMap.h"
#include <emmintrin.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo mb_emlrtRSI = {
    1431,                       /* lineNo */
    "MapLayer/getValueAllImpl", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo nb_emlrtRSI = {
    51,          /* lineNo */
    "circshift", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo ob_emlrtRSI = {
    96,                        /* lineNo */
    "circshift_multiple_dims", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo pb_emlrtRSI = {
    137,              /* lineNo */
    "circshift_core", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo qb_emlrtRSI = {
    133,              /* lineNo */
    "circshift_core", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo rb_emlrtRSI = {
    129,              /* lineNo */
    "circshift_core", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo sb_emlrtRSI = {
    124,              /* lineNo */
    "circshift_core", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo tb_emlrtRSI = {
    116,              /* lineNo */
    "circshift_core", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo ub_emlrtRSI = {
    112,              /* lineNo */
    "circshift_core", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo vb_emlrtRSI = {
    110,              /* lineNo */
    "circshift_core", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pathName
                                                                            */
};

static emlrtRSInfo cp_emlrtRSI = {
    1456,                                 /* lineNo */
    "MapLayer/getValueAtIndicesInternal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo dp_emlrtRSI = {
    1459,                                 /* lineNo */
    "MapLayer/getValueAtIndicesInternal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRSInfo ep_emlrtRSI = {
    1733,                             /* lineNo */
    "MapLayer/allocateExternalBlock", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

static emlrtRTEInfo j_emlrtRTEI = {
    38,          /* lineNo */
    48,          /* colNo */
    "circshift", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\elmat\\circshift.m" /* pName
                                                                            */
};

static emlrtECInfo eb_emlrtECI = {
    -1,                                   /* nDims */
    1451,                                 /* lineNo */
    24,                                   /* colNo */
    "MapLayer/getValueAtIndicesInternal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtECInfo fb_emlrtECI = {
    1,                                    /* nDims */
    1451,                                 /* lineNo */
    24,                                   /* colNo */
    "MapLayer/getValueAtIndicesInternal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtBCInfo ee_emlrtBCI = {
    -1,                                   /* iFirst */
    -1,                                   /* iLast */
    1460,                                 /* lineNo */
    21,                                   /* colNo */
    "",                                   /* aName */
    "MapLayer/getValueAtIndicesInternal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtBCInfo fe_emlrtBCI = {
    -1,                                   /* iFirst */
    -1,                                   /* iLast */
    1460,                                 /* lineNo */
    72,                                   /* colNo */
    "",                                   /* aName */
    "MapLayer/getValueAtIndicesInternal", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m", /* pName */
    0      /* checkKind */
};

static emlrtRTEInfo oe_emlrtRTEI = {
    1460,       /* lineNo */
    68,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo pe_emlrtRTEI = {
    1460,       /* lineNo */
    17,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo yh_emlrtRTEI = {
    1451,       /* lineNo */
    25,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo ai_emlrtRTEI = {
    1451,       /* lineNo */
    38,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo bi_emlrtRTEI = {
    1452,       /* lineNo */
    18,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo ci_emlrtRTEI = {
    1452,       /* lineNo */
    31,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo di_emlrtRTEI = {
    1440,       /* lineNo */
    24,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo ei_emlrtRTEI = {
    1451,       /* lineNo */
    13,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo fi_emlrtRTEI = {
    1451,       /* lineNo */
    24,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRTEInfo gi_emlrtRTEI = {
    1460,       /* lineNo */
    21,         /* colNo */
    "MapLayer", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pName */
};

static emlrtRSInfo vt_emlrtRSI = {
    1451,                                 /* lineNo */
    "MapLayer/getValueAtIndicesInternal", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\shared\\autonomous\\maplib\\internal\\+"
    "matlabshared\\+autonomous\\+internal\\MapLaye"
    "r.m" /* pathName */
};

/* Function Definitions */
void MapLayer_getValueAllImpl(const emlrtStack *sp, binaryOccupancyMap *obj,
                              boolean_T val[10000])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack st;
  real_T p[2];
  int32_T dim;
  int32_T i;
  int32_T j;
  int32_T k;
  boolean_T x[2];
  boolean_T exitg1;
  boolean_T overflow;
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
  x[0] = (obj->Index.Head[0] == 1.0);
  x[1] = (obj->Index.Head[1] == 1.0);
  overflow = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!x[k]) {
      overflow = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  if (overflow) {
    memcpy(&val[0], &obj->Buffer.Buffer[0], 10000U * sizeof(boolean_T));
  } else {
    __m128d r;
    int32_T absp[2];
    int32_T stride;
    int32_T u0;
    boolean_T buffer[50];
    st.site = &mb_emlrtRSI;
    memcpy(&val[0], &obj->Buffer.Buffer[0], 10000U * sizeof(boolean_T));
    r = _mm_loadu_pd(&obj->Index.Head[0]);
    _mm_storeu_pd(
        &p[0], _mm_mul_pd(_mm_sub_pd(r, _mm_set1_pd(1.0)), _mm_set1_pd(-1.0)));
    overflow = true;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 2)) {
      u0 = (int32_T)p[k];
      if ((u0 != p[k]) || (u0 == MIN_int32_T)) {
        overflow = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (!overflow) {
      emlrtErrorWithMessageIdR2018a(&st, &j_emlrtRTEI,
                                    "Coder:toolbox:circshift_InvalidShiftType",
                                    "Coder:toolbox:circshift_InvalidShiftType",
                                    6, 4, 5, "int32", 4, 5, "int32");
    }
    b_st.site = &nb_emlrtRSI;
    if (p[0] < 0.0) {
      u0 = -(int32_T)p[0];
      overflow = false;
    } else {
      u0 = (int32_T)p[0];
      overflow = true;
    }
    if (u0 > 100) {
      u0 -= 100 * (u0 / 100);
    }
    if (u0 > 50) {
      u0 = 100 - u0;
      overflow = !overflow;
    }
    absp[0] = u0;
    x[0] = overflow;
    if (p[1] < 0.0) {
      u0 = -(int32_T)p[1];
      overflow = false;
    } else {
      u0 = (int32_T)p[1];
      overflow = true;
    }
    if (u0 > 100) {
      u0 -= 100 * (u0 / 100);
    }
    if (u0 > 50) {
      u0 = 100 - u0;
      overflow = !overflow;
    }
    absp[1] = u0;
    x[1] = overflow;
    stride = 1;
    for (dim = 0; dim < 2; dim++) {
      int32_T npages;
      int32_T ns;
      int32_T pagesize;
      u0 = absp[dim];
      ns = u0 - 1;
      pagesize = stride * 100;
      c_st.site = &ob_emlrtRSI;
      npages = -99 * dim + 99;
      if (u0 > 0) {
        d_st.site = &vb_emlrtRSI;
        overflow = (stride > 2147483646);
        for (i = 0; i <= npages; i++) {
          int32_T pageroot;
          pageroot = i * pagesize;
          d_st.site = &ub_emlrtRSI;
          if (overflow) {
            e_st.site = &wb_emlrtRSI;
            check_forloop_overflow_error(&e_st);
          }
          for (j = 0; j < stride; j++) {
            int32_T i1;
            i1 = pageroot + j;
            if (x[dim]) {
              int32_T b_i;
              d_st.site = &tb_emlrtRSI;
              for (k = 0; k <= ns; k++) {
                buffer[k] = val[i1 + ((k - u0) + 100) * stride];
              }
              b_i = u0 + 1;
              for (k = 100; k >= b_i; k--) {
                val[i1 + (k - 1) * stride] = val[i1 + ((k - u0) - 1) * stride];
              }
              d_st.site = &sb_emlrtRSI;
              for (k = 0; k <= ns; k++) {
                val[i1 + k * stride] = buffer[k];
              }
            } else {
              int32_T b_i;
              d_st.site = &rb_emlrtRSI;
              for (k = 0; k <= ns; k++) {
                buffer[k] = val[i1 + k * stride];
              }
              b_i = 99 - u0;
              d_st.site = &qb_emlrtRSI;
              for (k = 0; k <= b_i; k++) {
                val[i1 + k * stride] = val[i1 + (k + u0) * stride];
              }
              d_st.site = &pb_emlrtRSI;
              for (k = 0; k <= ns; k++) {
                val[i1 + ((k - u0) + 100) * stride] = buffer[k];
              }
            }
          }
        }
      }
      stride = pagesize;
    }
    if (!obj->HasParent) {
      memcpy(&obj->Buffer.Buffer[0], &val[0], 10000U * sizeof(boolean_T));
      obj->Index.Head[0] = 1.0;
      obj->Index.Head[1] = 1.0;
    }
  }
}

void c_MapLayer_getValueAtIndicesInt(const emlrtStack *sp,
                                     const binaryOccupancyMap *obj,
                                     const emxArray_real_T *ind,
                                     emxArray_real_T *val)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_boolean_T *r;
  emxArray_boolean_T *validInd;
  emxArray_int32_T *r2;
  emxArray_int8_T *r4;
  emxArray_real_T *b_ind;
  const real_T *ind_data;
  real_T *b_ind_data;
  real_T *val_data;
  int32_T end;
  int32_T i;
  int32_T loop_ub;
  int32_T *r3;
  int8_T *r5;
  boolean_T *r1;
  boolean_T *validInd_data;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  ind_data = ind->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  emxInit_boolean_T(sp, &validInd, 1, &ei_emlrtRTEI);
  i = validInd->size[0];
  validInd->size[0] = ind->size[0];
  emxEnsureCapacity_boolean_T(sp, validInd, i, &yh_emlrtRTEI);
  validInd_data = validInd->data;
  loop_ub = ind->size[0];
  emxInit_boolean_T(sp, &r, 1, &fi_emlrtRTEI);
  i = r->size[0];
  r->size[0] = ind->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &ai_emlrtRTEI);
  r1 = r->data;
  for (i = 0; i < loop_ub; i++) {
    real_T d;
    d = ind_data[i];
    validInd_data[i] = (d > 0.0);
    r1[i] = (d < 101.0);
  }
  if (validInd->size[0] != r->size[0]) {
    emlrtSizeEqCheck1DR2012b(validInd->size[0], r->size[0], &eb_emlrtECI,
                             (emlrtConstCTX)sp);
  }
  loop_ub = validInd->size[0];
  for (i = 0; i < loop_ub; i++) {
    validInd_data[i] = (validInd_data[i] && r1[i]);
  }
  i = r->size[0];
  r->size[0] = ind->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &bi_emlrtRTEI);
  r1 = r->data;
  loop_ub = ind->size[0];
  for (i = 0; i < loop_ub; i++) {
    r1[i] = (ind_data[i + ind->size[0]] > 0.0);
  }
  if ((validInd->size[0] != r->size[0]) &&
      ((validInd->size[0] != 1) && (r->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(validInd->size[0], r->size[0], &fb_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (validInd->size[0] == r->size[0]) {
    loop_ub = validInd->size[0];
    for (i = 0; i < loop_ub; i++) {
      validInd_data[i] = (validInd_data[i] && r1[i]);
    }
  } else {
    st.site = &vt_emlrtRSI;
    d_and(&st, validInd, r);
    validInd_data = validInd->data;
  }
  i = r->size[0];
  r->size[0] = ind->size[0];
  emxEnsureCapacity_boolean_T(sp, r, i, &ci_emlrtRTEI);
  r1 = r->data;
  loop_ub = ind->size[0];
  for (i = 0; i < loop_ub; i++) {
    r1[i] = (ind_data[i + ind->size[0]] < 101.0);
  }
  if ((validInd->size[0] != r->size[0]) &&
      ((validInd->size[0] != 1) && (r->size[0] != 1))) {
    emlrtDimSizeImpxCheckR2021b(validInd->size[0], r->size[0], &fb_emlrtECI,
                                (emlrtConstCTX)sp);
  }
  if (validInd->size[0] == r->size[0]) {
    loop_ub = validInd->size[0];
    for (i = 0; i < loop_ub; i++) {
      validInd_data[i] = (validInd_data[i] && r1[i]);
    }
  } else {
    st.site = &vt_emlrtRSI;
    d_and(&st, validInd, r);
    validInd_data = validInd->data;
  }
  st.site = &cp_emlrtRSI;
  b_st.site = &ep_emlrtRSI;
  b_repmat(&b_st, ind->size[0], val);
  val_data = val->data;
  st.site = &dp_emlrtRSI;
  if (any(validInd)) {
    end = validInd->size[0] - 1;
    loop_ub = 0;
    for (i = 0; i <= end; i++) {
      if (validInd_data[i]) {
        loop_ub++;
      }
    }
    emxInit_int32_T(sp, &r2, 1, &gi_emlrtRTEI);
    i = r2->size[0];
    r2->size[0] = loop_ub;
    emxEnsureCapacity_int32_T(sp, r2, i, &di_emlrtRTEI);
    r3 = r2->data;
    loop_ub = 0;
    for (i = 0; i <= end; i++) {
      if (validInd_data[i]) {
        r3[loop_ub] = i;
        loop_ub++;
      }
    }
    loop_ub = r2->size[0];
    for (i = 0; i < loop_ub; i++) {
      if (r3[i] > val->size[0] - 1) {
        emlrtDynamicBoundsCheckR2012b(r3[i], 0, val->size[0] - 1, &ee_emlrtBCI,
                                      (emlrtConstCTX)sp);
      }
    }
    emxInit_real_T(sp, &b_ind, 2, &oe_emlrtRTEI);
    i = b_ind->size[0] * b_ind->size[1];
    b_ind->size[0] = r2->size[0];
    b_ind->size[1] = 2;
    emxEnsureCapacity_real_T(sp, b_ind, i, &oe_emlrtRTEI);
    b_ind_data = b_ind->data;
    loop_ub = r2->size[0];
    for (i = 0; i < 2; i++) {
      for (end = 0; end < loop_ub; end++) {
        if (r3[end] > ind->size[0] - 1) {
          emlrtDynamicBoundsCheckR2012b(r3[end], 0, ind->size[0] - 1,
                                        &fe_emlrtBCI, (emlrtConstCTX)sp);
        }
        b_ind_data[end + b_ind->size[0] * i] =
            ind_data[r3[end] + ind->size[0] * i];
      }
    }
    st.site = &ch_emlrtRSI;
    c_CircularBuffer_getValueAtIndi(&st, &obj->Buffer, b_ind, r);
    r1 = r->data;
    emxFree_real_T(sp, &b_ind);
    emxInit_int8_T(sp, &r4, &pe_emlrtRTEI);
    i = r4->size[0];
    r4->size[0] = r->size[0];
    emxEnsureCapacity_int8_T(sp, r4, i, &pe_emlrtRTEI);
    r5 = r4->data;
    loop_ub = r->size[0];
    for (i = 0; i < loop_ub; i++) {
      r5[i] = (int8_T)r1[i];
    }
    emlrtSubAssignSizeCheckR2012b(&r2->size[0], 1, &r4->size[0], 1, &h_emlrtECI,
                                  (emlrtCTX)sp);
    loop_ub = r4->size[0];
    for (i = 0; i < loop_ub; i++) {
      val_data[r3[i]] = r5[i];
    }
    emxFree_int32_T(sp, &r2);
    emxFree_int8_T(sp, &r4);
  }
  emxFree_boolean_T(sp, &r);
  emxFree_boolean_T(sp, &validInd);
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (MapLayer.c) */
