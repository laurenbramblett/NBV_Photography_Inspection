/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * unique.c
 *
 * Code generation for function 'unique'
 *
 */

/* Include files */
#include "unique.h"
#include "codegenPathPlanner_emxutil.h"
#include "codegenPathPlanner_types.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo rp_emlrtRSI = {
    164,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo sp_emlrtRSI = {
    166,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo tp_emlrtRSI = {
    183,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo up_emlrtRSI = {
    210,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo vp_emlrtRSI = {
    223,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo wp_emlrtRSI = {
    234,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo xp_emlrtRSI = {
    248,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRSInfo
    yp_emlrtRSI =
        {
            145,       /* lineNo */
            "sortIdx", /* fcnName */
            "C:\\Program "
            "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
            "internal\\sortIdx.m" /* pathName */
};

static emlrtRSInfo xr_emlrtRSI = {
    242,             /* lineNo */
    "unique_vector", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pathName
                                                                       */
};

static emlrtRTEInfo pc_emlrtRTEI = {
    241,             /* lineNo */
    1,               /* colNo */
    "unique_vector", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static emlrtRTEInfo bj_emlrtRTEI = {
    164,      /* lineNo */
    1,        /* colNo */
    "unique", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static emlrtRTEInfo cj_emlrtRTEI =
    {
        52,          /* lineNo */
        9,           /* colNo */
        "mergesort", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\mergesort.m" /* pName */
};

static emlrtRTEInfo dj_emlrtRTEI = {
    165,      /* lineNo */
    20,       /* colNo */
    "unique", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static emlrtRTEInfo ej_emlrtRTEI = {
    242,      /* lineNo */
    1,        /* colNo */
    "unique", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2023a\\toolbox\\eml\\lib\\matlab\\ops\\unique.m" /* pName
                                                                       */
};

static emlrtRTEInfo fj_emlrtRTEI =
    {
        52,          /* lineNo */
        1,           /* colNo */
        "mergesort", /* fName */
        "C:\\Program "
        "Files\\MATLAB\\R2023a\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\mergesort.m" /* pName */
};

/* Function Definitions */
int32_T b_unique_vector(const emlrtStack *sp, const real_T a_data[],
                        int32_T a_size, real_T b_data[])
{
  emlrtStack st;
  real_T x;
  int32_T idx_data[4];
  int32_T iwork_data[4];
  int32_T iv[2];
  int32_T b_size;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T k;
  int32_T nInf;
  int32_T qEnd;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  if (a_size - 1 >= 0) {
    memset(&idx_data[0], 0, (uint32_T)a_size * sizeof(int32_T));
  }
  if (a_size != 0) {
    nInf = a_size - 1;
    for (k = 1; k <= nInf; k += 2) {
      x = a_data[k];
      if ((a_data[k - 1] <= x) || muDoubleScalarIsNaN(x)) {
        idx_data[k - 1] = k;
        idx_data[k] = k + 1;
      } else {
        idx_data[k - 1] = k + 1;
        idx_data[k] = k;
      }
    }
    if ((a_size & 1) != 0) {
      idx_data[a_size - 1] = a_size;
    }
    i = 2;
    while (i < a_size) {
      i2 = i << 1;
      j = 1;
      for (b_size = i + 1; b_size < a_size + 1; b_size = qEnd + i) {
        int32_T kEnd;
        int32_T p;
        int32_T q;
        p = j;
        q = b_size - 1;
        qEnd = j + i2;
        if (qEnd > a_size + 1) {
          qEnd = a_size + 1;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          x = a_data[idx_data[q] - 1];
          nInf = idx_data[p - 1];
          if ((a_data[nInf - 1] <= x) || muDoubleScalarIsNaN(x)) {
            iwork_data[k] = nInf;
            p++;
            if (p == b_size) {
              while (q + 1 < qEnd) {
                k++;
                iwork_data[k] = idx_data[q];
                q++;
              }
            }
          } else {
            iwork_data[k] = idx_data[q];
            q++;
            if (q + 1 == qEnd) {
              while (p < b_size) {
                k++;
                iwork_data[k] = idx_data[p - 1];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx_data[(j + k) - 1] = iwork_data[k];
        }
        j = qEnd;
      }
      i = i2;
    }
  }
  for (k = 0; k < a_size; k++) {
    b_data[k] = a_data[idx_data[k] - 1];
  }
  k = 0;
  while ((k + 1 <= a_size) && muDoubleScalarIsInf(b_data[k]) &&
         (b_data[k] < 0.0)) {
    k++;
  }
  i = k;
  k = a_size;
  while ((k >= 1) && muDoubleScalarIsNaN(b_data[k - 1])) {
    k--;
  }
  i2 = a_size - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    x = b_data[k - 1];
    if (muDoubleScalarIsInf(x) && (x > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  nInf = (a_size - k) - i2;
  b_size = 0;
  if (i > 0) {
    b_size = 1;
  }
  while (i + 1 <= k) {
    x = b_data[i];
    do {
      i++;
    } while (!((i + 1 > k) || (b_data[i] != x)));
    b_size++;
    b_data[b_size - 1] = x;
  }
  if (nInf > 0) {
    b_size++;
    b_data[b_size - 1] = b_data[k];
  }
  i = k + nInf;
  for (j = 0; j < i2; j++) {
    b_data[b_size + j] = b_data[i + j];
  }
  if (i2 - 1 >= 0) {
    b_size += i2;
  }
  if (b_size > a_size) {
    emlrtErrorWithMessageIdR2018a(sp, &pc_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  if (b_size < 1) {
    b_size = 0;
  }
  iv[0] = 1;
  iv[1] = b_size;
  st.site = &xr_emlrtRSI;
  indexShapeCheck(&st, a_size, iv);
  return b_size;
}

void unique_vector(const emlrtStack *sp, const emxArray_real_T *a,
                   emxArray_real_T *b)
{
  emlrtStack b_st;
  emlrtStack st;
  emxArray_int32_T *idx;
  emxArray_int32_T *iwork;
  const real_T *a_data;
  real_T x;
  real_T *b_data;
  int32_T b_i;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T k;
  int32_T n;
  int32_T na;
  int32_T nb;
  int32_T pEnd;
  int32_T qEnd;
  int32_T *idx_data;
  int32_T *iwork_data;
  boolean_T exitg1;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  a_data = a->data;
  emlrtHeapReferenceStackEnterFcnR2012b((emlrtConstCTX)sp);
  na = a->size[1];
  st.site = &rp_emlrtRSI;
  n = a->size[1] + 1;
  emxInit_int32_T(&st, &idx, 2, &bj_emlrtRTEI);
  i = idx->size[0] * idx->size[1];
  idx->size[0] = 1;
  idx->size[1] = a->size[1];
  emxEnsureCapacity_int32_T(&st, idx, i, &bj_emlrtRTEI);
  idx_data = idx->data;
  b_i = a->size[1];
  for (i = 0; i < b_i; i++) {
    idx_data[i] = 0;
  }
  b_st.site = &yp_emlrtRSI;
  emxInit_int32_T(&b_st, &iwork, 1, &fj_emlrtRTEI);
  i = iwork->size[0];
  iwork->size[0] = a->size[1];
  emxEnsureCapacity_int32_T(&b_st, iwork, i, &cj_emlrtRTEI);
  iwork_data = iwork->data;
  i = a->size[1] - 1;
  for (k = 1; k <= i; k += 2) {
    x = a_data[k];
    if ((a_data[k - 1] <= x) || muDoubleScalarIsNaN(x)) {
      idx_data[k - 1] = k;
      idx_data[k] = k + 1;
    } else {
      idx_data[k - 1] = k + 1;
      idx_data[k] = k;
    }
  }
  if ((a->size[1] & 1) != 0) {
    idx_data[a->size[1] - 1] = a->size[1];
  }
  b_i = 2;
  while (b_i < n - 1) {
    i2 = b_i << 1;
    j = 1;
    for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
      int32_T kEnd;
      int32_T q;
      nb = j;
      q = pEnd - 1;
      qEnd = j + i2;
      if (qEnd > n) {
        qEnd = n;
      }
      k = 0;
      kEnd = qEnd - j;
      while (k + 1 <= kEnd) {
        x = a_data[idx_data[q] - 1];
        i = idx_data[nb - 1];
        if ((a_data[i - 1] <= x) || muDoubleScalarIsNaN(x)) {
          iwork_data[k] = i;
          nb++;
          if (nb == pEnd) {
            while (q + 1 < qEnd) {
              k++;
              iwork_data[k] = idx_data[q];
              q++;
            }
          }
        } else {
          iwork_data[k] = idx_data[q];
          q++;
          if (q + 1 == qEnd) {
            while (nb < pEnd) {
              k++;
              iwork_data[k] = idx_data[nb - 1];
              nb++;
            }
          }
        }
        k++;
      }
      for (k = 0; k < kEnd; k++) {
        idx_data[(j + k) - 1] = iwork_data[k];
      }
      j = qEnd;
    }
    b_i = i2;
  }
  emxFree_int32_T(&b_st, &iwork);
  i = b->size[0] * b->size[1];
  b->size[0] = 1;
  b->size[1] = a->size[1];
  emxEnsureCapacity_real_T(sp, b, i, &dj_emlrtRTEI);
  b_data = b->data;
  st.site = &sp_emlrtRSI;
  for (k = 0; k < na; k++) {
    b_data[k] = a_data[idx_data[k] - 1];
  }
  emxFree_int32_T(sp, &idx);
  k = 0;
  while ((k + 1 <= na) && muDoubleScalarIsInf(b_data[k]) && (b_data[k] < 0.0)) {
    k++;
  }
  i2 = k;
  k = a->size[1];
  while ((k >= 1) && muDoubleScalarIsNaN(b_data[k - 1])) {
    k--;
  }
  pEnd = a->size[1] - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    x = b_data[k - 1];
    if (muDoubleScalarIsInf(x) && (x > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  b_i = (a->size[1] - k) - pEnd;
  nb = 0;
  if (i2 > 0) {
    nb = 1;
    st.site = &tp_emlrtRSI;
  }
  while (i2 + 1 <= k) {
    x = b_data[i2];
    do {
      i2++;
    } while (!((i2 + 1 > k) || (b_data[i2] != x)));
    nb++;
    b_data[nb - 1] = x;
    st.site = &up_emlrtRSI;
  }
  if (b_i > 0) {
    nb++;
    b_data[nb - 1] = b_data[k];
    st.site = &vp_emlrtRSI;
  }
  i2 = k + b_i;
  st.site = &wp_emlrtRSI;
  for (j = 0; j < pEnd; j++) {
    b_data[nb + j] = b_data[i2 + j];
  }
  if (pEnd - 1 >= 0) {
    nb += pEnd;
  }
  if (nb > a->size[1]) {
    emlrtErrorWithMessageIdR2018a(sp, &pc_emlrtRTEI,
                                  "Coder:builtins:AssertionFailed",
                                  "Coder:builtins:AssertionFailed", 0);
  }
  i = b->size[0] * b->size[1];
  if (nb < 1) {
    b->size[1] = 0;
  } else {
    b->size[1] = nb;
  }
  emxEnsureCapacity_real_T(sp, b, i, &ej_emlrtRTEI);
  st.site = &xp_emlrtRSI;
  emlrtHeapReferenceStackLeaveFcnR2012b((emlrtConstCTX)sp);
}

/* End of code generation (unique.c) */
