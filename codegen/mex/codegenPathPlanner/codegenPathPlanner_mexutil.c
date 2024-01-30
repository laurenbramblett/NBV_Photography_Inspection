/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * codegenPathPlanner_mexutil.c
 *
 * Code generation for function 'codegenPathPlanner_mexutil'
 *
 */

/* Include files */
#include "codegenPathPlanner_mexutil.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                        const emlrtMsgIdentifier *parentId, char_T y[23])
{
  i_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

const mxArray *b_sprintf(const emlrtStack *sp, const mxArray *m1,
                         const mxArray *m2, emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m;
  pArrays[0] = m1;
  pArrays[1] = m2;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m, 2, &pArrays[0],
                               "sprintf", true, location);
}

void emlrt_marshallIn(const emlrtStack *sp,
                      const mxArray *a__output_of_sprintf_,
                      const char_T *identifier, char_T y[23])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_sprintf_), &thisId, y);
  emlrtDestroyArray(&a__output_of_sprintf_);
}

void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                        const emlrtMsgIdentifier *msgId, char_T ret[23])
{
  static const int32_T dims[2] = {1, 23};
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "char", false, 2U,
                          (const void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtConstCTX)sp, src, &ret[0], 23);
  emlrtDestroyArray(&src);
}

/* End of code generation (codegenPathPlanner_mexutil.c) */
